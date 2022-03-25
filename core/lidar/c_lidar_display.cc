/*
 * c_lidar_display.cc
 *
 *  Created on: Mar 19, 2022
 *      Author: amyznikov
 */

#include "c_lidar_display.h"
#include <core/proc/histogram.h>
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member* members_of<LIDAR_DISPLAY_TYPE>()
{
  static constexpr c_enum_member members[] = {
      { LIDAR_DISPLAY_DEPTH, "DEPTH", "" },
      { LIDAR_DISPLAY_DISTANCE, "DISTANCE", "" },
      { LIDAR_DISPLAY_INTENSITY, "INTENSITY", "" },
      { LIDAR_DISPLAY_HEIGHT, "HEIGHT", "" },
      { LIDAR_DISPLAY_AZIMUTH, "AZIMUTH", "" },
      { LIDAR_DISPLAY_ELEVATION, "ELEVATION", "" },
      { LIDAR_DISPLAY_TIMESTAMP, "TIMESTAMP", "" },
      { LIDAR_DISPLAY_PACKET, "PACKET", "" },
      { LIDAR_DISPLAY_DATABLOCK, "DATABLOCK", "" },
      { LIDAR_DISPLAY_LASERID, "LASERID", "" },
      { LIDAR_DISPLAY_ALIASING, "ALIASING", "" },
      { LIDAR_DISPLAY_DISTANCE, nullptr},
  };

  return members;
}

///////////////////////////////////////////////////////////////////////////////


/**
 * For given lidar point p compute row and column in range image
 * using specified azimuthal resolution and image size
 * */
static inline bool project_to_range_image(const c_lidar_point & p,
    const cv::Size & image_size,
    double azimuthal_resolution_radians_per_pixel,
    int * outr,
    int * outc)
{
  const int r = image_size.height - p.laser_ring - 1;
  if( r >= 0 && r < image_size.height ) {
    int c = (int) (p.azimuth / azimuthal_resolution_radians_per_pixel);
    if( c >= 0 && c < image_size.width ) {
      *outr = r;
      *outc = c;
      return true;
    }
  }
  return false;
}

static inline double compute_depth(const c_lidar_point & p)
{
  if ( p.distance > 0 ) {
    const double x = p.distance * cos(p.elevation) * sin(p.azimuth);
    const double y = p.distance * cos(p.elevation) * cos(p.azimuth);
    return sqrt(x * x + y * y);
  }
  return 0;
}

static inline double compute_intensity(const c_lidar_point & p)
{
  return p.intensity;
}

static inline double compute_height(const c_lidar_point & p)
{
  return p.distance > 0 ? p.distance * sin(p.elevation) : 0;
}

static inline double compute_minimal_timestamp(const c_lidar_frame * frame)
{
  double mints = frame->points.front().timestamp;
  for ( const c_lidar_point & p : frame->points ) {
    if ( p.timestamp < mints ) {
      mints = p.timestamp;
    }
  }
  return mints;
}


namespace {
  typedef std::function<double(const c_lidar_point&)> LidarDataFunc;
}

static const std::map<LIDAR_DISPLAY_TYPE, LidarDataFunc> display_to_lidar_data_mapping = {

    { LIDAR_DISPLAY_DEPTH,
        LidarDataFunc([](const c_lidar_point & p) {
          return compute_depth(p);
        })
    },

    { LIDAR_DISPLAY_DISTANCE,
        LidarDataFunc([](const c_lidar_point & p) {
          return p.distance;
        })
    },

    { LIDAR_DISPLAY_INTENSITY,
        LidarDataFunc([](const c_lidar_point & p) {
          return compute_intensity(p);
        })
    },

    { LIDAR_DISPLAY_HEIGHT,
        LidarDataFunc([](const c_lidar_point & p) {
          return compute_height(p);
        })
    },

    { LIDAR_DISPLAY_AZIMUTH,
        LidarDataFunc([](const c_lidar_point & p) {
          return p.azimuth * 180 / CV_PI;
        })
    },

    { LIDAR_DISPLAY_ELEVATION,
        LidarDataFunc([](const c_lidar_point & p) {
          return p.elevation * 180 / CV_PI;
        })
    },

    { LIDAR_DISPLAY_TIMESTAMP,
        LidarDataFunc([](const c_lidar_point & p) {
          return p.timestamp;
        })
    },


    { LIDAR_DISPLAY_LASERID,
        LidarDataFunc([](const c_lidar_point & p) {
          return p.laser_id;
        })
    },

    { LIDAR_DISPLAY_DATABLOCK,
        LidarDataFunc([](const c_lidar_point & p) {
          return p.datablock;
        })
    },

    { LIDAR_DISPLAY_PACKET,
        LidarDataFunc([](const c_lidar_point & p) {
          return p.pkt;
        })
    },

};

///////////////////////////////////////////////////////////////////////////////
c_lidar_display::c_lidar_display()
{
  mtf_.set_output_range(0, 255);
}

void c_lidar_display::set_display_type(LIDAR_DISPLAY_TYPE v)
{
  display_type_ = v;
}

LIDAR_DISPLAY_TYPE c_lidar_display::display_type() const
{
  return display_type_;
}

void c_lidar_display::set_azimuthal_resolution(double degrees_per_pixel)
{
  azimuthal_resolution_ = degrees_per_pixel;
}

double c_lidar_display::azimuthal_resolution() const
{
  return azimuthal_resolution_;
}

void c_lidar_display::set_num_lasers(int v)
{
  num_lasers_ = v;
}

int c_lidar_display::num_lasers() const
{
  return num_lasers_;
}

void c_lidar_display::set_colormap(COLORMAP v)
{
  if( (colormap_ = v) == COLORMAP_NONE ) {
    lut_.release();
  }
  else {
    cv::Mat1b M(1, 256);
    for( int i = 0; i < 256; ++i ) {
      M[0][i] = i;
    }
    apply_colormap(M, lut_, colormap_);
  }
}

COLORMAP c_lidar_display::colormap() const
{
  return colormap_;
}

c_pixinsight_mtf & c_lidar_display::mtf()
{
  return mtf_;
}

const c_pixinsight_mtf & c_lidar_display::mtf() const
{
  return mtf_;
}

void c_lidar_display::compute_input_data_range(const c_lidar_frame * frame,
    double * minval, double * maxval) const
{
  if( !frame || frame->points.empty() ) {
    *minval = 0;
    *maxval = 1;
    return;
  }

  if( display_type_ == LIDAR_DISPLAY_ALIASING ) {

    cv::Mat1w image;

    create_aliasing_image_(frame, image, cv::noArray());

    cv::minMaxLoc(image, minval, maxval);

    return;
  }


  const auto m = display_to_lidar_data_mapping.find(display_type_);
  if( m != display_to_lidar_data_mapping.end() ) {

    const LidarDataFunc &func = m->second;

    *minval = *maxval = func(frame->points.front());

    for( const c_lidar_point &p : frame->points ) {
      const double value = func(p);
      if( value < *minval ) {
        *minval = value;
      }
      else if( value > *maxval ) {
        *maxval = value;
      }
    }

    if( display_type_ == LIDAR_DISPLAY_TIMESTAMP ) {
      *maxval -= *minval;
      *minval = 0;
    }
  }
  else {
    *minval = 0;
    *maxval = 1;
    CF_ERROR("APP BUG: Mapping to data function is not implemented for display_type_=%s (%d)",
        toString(display_type_), (int)display_type_);
  }
}

void c_lidar_display::create_aliasing_image_(const c_lidar_frame * frame,
    cv::OutputArray output_range_image,
    cv::OutputArray output_display_image) const
{
  if( !frame || num_lasers_ < 1 || azimuthal_resolution_ <= 0 ) {
    if( output_range_image.needed() ) {
      output_range_image.release();
    }
    if( output_display_image.needed() ) {
      output_display_image.release();
    }
    return;
  }

  const double radians_per_pixel = azimuthal_resolution_ * CV_PI / 180;
  const cv::Size image_size((int) (2 * CV_PI / radians_per_pixel), num_lasers_);

  cv::Mat1w image;
  cv::Mat3b display_image;
  int r, c;

  if( output_range_image.needed() || output_display_image.needed() ) {
    image.create(image_size);
    image.setTo(0);
  }

  for( const c_lidar_point &p : frame->points ) {
    if( project_to_range_image(p, image_size, radians_per_pixel, &r, &c) ) {
      if( !image.empty() ) {
        image[r][c]++;
      }
    }
  }

  if( output_display_image.needed() ) {

    display_image.create(image_size);
    display_image.setTo(0);

    for( int y = 0; y < image.rows; ++y ) {
      for( int x = 0; x < image.cols; ++x ) {

        const uint8_t v = mtf_.apply(image[y][x]);
        if( lut_.empty() ) {
          display_image[y][x] = cv::Vec3b(v, v, v);
        }
        else {
          display_image[y][x] = lut_[0][v];
        }
      }
    }

    if( output_display_image.fixedType() && output_display_image.type() != display_image.type() ) {
      display_image.convertTo(output_display_image, output_display_image.type());
    }
    else {
      output_display_image.move(display_image);
    }
  }

  if( output_range_image.needed() ) {
    if( output_range_image.fixedType() && output_range_image.type() != image.type() ) {
      image.convertTo(output_range_image, output_range_image.type());
    }
    else {
      output_range_image.move(image);
    }
  }
}


void c_lidar_display::create_range_image(const c_lidar_frame * frame,
    cv::OutputArray output_range_image,
    cv::OutputArray output_display_image,
    cv::OutputArray output_mask) const
{
  if( !frame || frame->points.empty() || num_lasers_ < 1 || azimuthal_resolution_ <= 0 ) {
    if( output_range_image.needed() ) {
      output_range_image.release();
    }
    if( output_display_image.needed() ) {
      output_display_image.release();
    }
    if( output_mask.needed() ) {
      output_mask.release();
    }
    return;
  }

  const double radians_per_pixel = azimuthal_resolution_ * CV_PI / 180;
  const cv::Size image_size((int) (2 * CV_PI / radians_per_pixel), num_lasers_);

  cv::Mat1f range_image;
  cv::Mat3b display_image;
  cv::Mat1b mask;
  cv::Mat1f distances;
  int r, c;

  if( output_range_image.needed() ) {
    if( display_type_ != LIDAR_DISPLAY_DEPTH && display_type_ != LIDAR_DISPLAY_DISTANCE ) {
      range_image.create(image_size);
      range_image.setTo(0);
    }
  }

  if( output_mask.needed() ) {
    mask.create(image_size);
    mask.setTo(0);
  }

  if( output_display_image.needed() ) {
    display_image.create(image_size);
    display_image.setTo(0);
  }

  const auto m = display_to_lidar_data_mapping.find(display_type_);
  if( m != display_to_lidar_data_mapping.end() ) {

    double bias = 0;
    if ( display_type_ == LIDAR_DISPLAY_TIMESTAMP ) {
      bias = compute_minimal_timestamp(frame);
    }

    distances.create(image_size);
    distances.setTo(0);

    const LidarDataFunc & func = m->second;

    for( const c_lidar_point &p : frame->points ) {
      if( project_to_range_image(p, image_size, radians_per_pixel, &r, &c) ) {

        const double value = func(p) - bias;

        const double distance =
            display_type_ == LIDAR_DISPLAY_DEPTH || display_type_ == LIDAR_DISPLAY_DISTANCE ?
                value :
                p.distance;

        if ( distance > 0 ) {
          if ( distances[r][c] > 0 && distance > distances[r][c] ) {
            continue;
          }
          distances[r][c] = distance;
        }

        if( !range_image.empty() ) {
          range_image[r][c] = value;
        }

        if ( !mask.empty() ) {
          mask[r][c] = 255;
        }

        if( !display_image.empty() ) {

          const uint8_t color = mtf_.apply(value);

          if( lut_.empty() ) {
            display_image[r][c] = cv::Vec3b(color, color, color);
          }
          else {
            display_image[r][c] = lut_[0][color];
          }
        }
      }
    }
  }
  else if( display_type_ == LIDAR_DISPLAY_ALIASING ) {

    output_mask.release();
    create_aliasing_image_(frame, output_range_image, output_display_image);

    return;
  }
  else {
    CF_ERROR("Invalid or not supported display_type_=%d (%s) requested",
        (int )display_type_,
        toString(display_type_));
  }

  if( output_range_image.needed() ) {

    if ( display_type_ == LIDAR_DISPLAY_DEPTH || display_type_ == LIDAR_DISPLAY_DISTANCE ) {
      range_image = distances;
    }

    if( output_range_image.fixedType() && output_range_image.type() != range_image.type() ) {
      range_image.convertTo(output_range_image, output_range_image.type());
    }
    else {
      output_range_image.move(range_image);
    }
  }

  if( output_display_image.needed() ) {
    if( output_display_image.fixedType() && output_display_image.type() != display_image.type() ) {
      display_image.convertTo(output_display_image, output_display_image.type());
    }
    else {
      output_display_image.move(display_image);
    }
  }

  if( output_mask.needed() ) {
    if( output_mask.fixedType() && output_mask.type() != mask.type() ) {
      mask.convertTo(output_mask, output_mask.type());
    }
    else {
      output_mask.move(mask);
    }
  }
}


void c_lidar_display::create_point_cloud(const c_lidar_frame * frame,
    std::vector<cv::Vec3f> * positions,
    std::vector<cv::Vec3b> * colors)
{
  if( positions ) {
    positions->clear();
    if( frame ) {
      convert_to_cartesian(frame, *positions);
    }
  }

  if( colors ) {

    colors->clear();

    if( frame ) {

      colors->reserve(frame->points.size());

      const auto m = display_to_lidar_data_mapping.find(display_type_);
      if( m != display_to_lidar_data_mapping.end() ) {

        const LidarDataFunc & func = m->second;

        double bias = 0;
        if ( display_type_ == LIDAR_DISPLAY_TIMESTAMP ) {
          bias = compute_minimal_timestamp(frame);
        }

        for( const c_lidar_point &p : frame->points ) {
          const uint8_t color = mtf_.apply(func(p) -  bias);
          if( lut_.empty() ) {
            colors->emplace_back(color, color, color);
          }
          else {
            colors->emplace_back(lut_[0][color]);
          }
        }
      }
      else if( display_type_ == LIDAR_DISPLAY_ALIASING ) {

        cv::Mat3b display_image;

        create_aliasing_image_(frame, cv::noArray(), display_image);

        const double radians_per_pixel = azimuthal_resolution_ * CV_PI / 180;
        const cv::Size image_size = display_image.size();
        int r, c;

        for( const c_lidar_point &p : frame->points ) {
          if( project_to_range_image(p, display_image.size(), radians_per_pixel, &r, &c) ) {
            colors->emplace_back(display_image[r][c]);
          }
        }
      }
      else {
        const cv::Vec3b color = lut_.empty() ? cv::Vec3b::all(255) : lut_[0][255];
        for( const c_lidar_point &p : frame->points ) {
          colors->emplace_back(color);
        }
      }
    }
  }
}


void c_lidar_display::create_histogramm_(const c_lidar_frame * frame,
    double hmin, double hmax, int nbins, int channels,
    cv::OutputArray H,
    const c_pixinsight_mtf * mtf) const
{
  c_histogram_builder builder;
  cv::Scalar s;

  builder.set_input_range(hmin, hmax);
  builder.set_bins(nbins);
  builder.set_channels(1);

  const auto m = display_to_lidar_data_mapping.find(display_type_);

  if( m != display_to_lidar_data_mapping.end() ) {

    const LidarDataFunc & func = m->second;

    double bias = 0;
    if ( display_type_ == LIDAR_DISPLAY_TIMESTAMP ) {
      bias = compute_minimal_timestamp(frame);
    }

    for( const c_lidar_point &p : frame->points ) {
      s[0] = func(p) - bias;
      if ( mtf ) {
        s[0] = mtf->apply(s[0]);
      }
      builder.add_pixel(s);
    }

  }
  else if (display_type_ == LIDAR_DISPLAY_ALIASING ) {

    cv::Mat1w image;
    create_aliasing_image_(frame, image, cv::noArray());

    for( int y = 0; y < image.rows; ++y ) {
      for( int x = 0; x < image.cols; ++x ) {
        s[0] = image[y][x];
        if ( mtf ) {
          s[0] = mtf->apply(s[0]);
        }
        builder.add_pixel(s);
      }
    }
  }
  else {
    CF_ERROR("APP BUG: Mapping to data function is not implemented for display_type_=%s (%d)",
        toString(display_type_), (int)display_type_);
  }

  builder.compute(H);
}


void c_lidar_display::create_input_histogramm(const c_lidar_frame * frame,
    cv::OutputArray H,
    double * hmin, double * hmax)
{
  compute_input_data_range(frame, hmin, hmax);

  if( !frame || frame->points.empty() ) {
    H.release();
  }
  else {
    create_histogramm_(frame, *hmin, *hmax, 256, 1, H);
  }
}


void c_lidar_display::create_output_histogramm(const c_lidar_frame * frame,
    cv::OutputArray H,
    double * hmin, double * hmax)
{
  mtf_.get_output_range(hmin, hmax);

  if( !frame || frame->points.empty() ) {
    H.release();
  }
  else {
    create_histogramm_(frame, *hmin, *hmax, 256, 1, H, &mtf_);
  }
}

