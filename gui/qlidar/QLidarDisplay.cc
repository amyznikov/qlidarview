/*
 * QLidarDisplayFunction.cc
 *
 *  Created on: Mar 19, 2022
 *      Author: amyznikov
 */
#include "QLidarDisplay.h"
#include <gui/widgets/settings.h>

QLidarDisplay::QLidarDisplay(LIDAR_DISPLAY_TYPE display_type, QObject * parent)
  : QObject(parent)
{
  display_.set_display_type(display_type);
}

LIDAR_DISPLAY_TYPE QLidarDisplay::type() const
{
  return display_.display_type();
}

void QLidarDisplay::set_azimuthal_resolution(double degrees_per_pixel)
{
  display_.set_azimuthal_resolution(degrees_per_pixel);
  emit update();
}

double QLidarDisplay::azimuthal_resolution() const
{
  return display_.azimuthal_resolution();
}

void QLidarDisplay::set_start_azimuth(double start_azimuth_in_degrees)
{
  display_.set_start_azimuth(start_azimuth_in_degrees);
  emit update();
}

double QLidarDisplay::start_azimuth() const
{
  return display_.start_azimuth();
}

void QLidarDisplay::set_num_lasers(int n)
{
  display_.set_num_lasers(n);
}

int QLidarDisplay::num_lasers() const
{
  return display_.num_lasers();
}

void QLidarDisplay::set_colormap(COLORMAP v)
{
  display_.set_colormap(v);
  emit update();
}

COLORMAP QLidarDisplay::colormap() const
{
  return display_.colormap();
}

void QLidarDisplay::set_current_frame(c_lidar_frame::sptr frame)
{
  current_frame_ = frame;
  emit update();
}

const c_lidar_frame::sptr & QLidarDisplay::current_frame() const
{
  return current_frame_;
}

c_pixinsight_mtf & QLidarDisplay::mtf()
{
  return display_.mtf();
}

const c_pixinsight_mtf & QLidarDisplay::mtf() const
{
  return display_.mtf();
}

void QLidarDisplay::create_range_image(cv::OutputArray output_range_image, cv::OutputArray output_display_image,
    cv::OutputArray output_mask)
{
  display_.create_range_image(current_frame_.get(),
      output_range_image,
      output_display_image,
      output_mask);
}

void QLidarDisplay::create_point_cloud(std::vector<cv::Vec3f> * positions, std::vector<cv::Vec3b> * colors)
{
  display_.create_point_cloud(current_frame_.get(),
      positions,
      colors);
}

void QLidarDisplay::create_input_histogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  display_.create_input_histogramm(current_frame_.get(), H, hmin, hmax);
}


void QLidarDisplay::create_output_histogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  display_.create_output_histogramm(current_frame_.get(), H, hmin, hmax);
}

void QLidarDisplay::compute_input_data_range(double * minval, double * maxval) const
{
  display_.compute_input_data_range(current_frame_.get(), minval, maxval);
}


void QLidarDisplay::load_parameters()
{
  const QString prefix =
      QString("LIDAR_DISPLAY_%1").arg(toString(display_.display_type()));

  QSettings settings;
  double min, max;

  display_.mtf().get_input_range(&min, &max);

  min = settings.value(QString("%1/imin").arg(prefix), min).toDouble();
  max = settings.value(QString("%1/imax").arg(prefix), max).toDouble();
  display_.mtf().set_input_range(min, max);

  display_.set_colormap(fromString(settings.value(QString("%1/cmap").arg(prefix)).
      toString().toStdString(), display_.colormap()));
}

void QLidarDisplay::save_paramters()
{
  const QString prefix =
      QString("LIDAR_DISPLAY_%1").arg(toString(
          display_.display_type()));

  QSettings settings;
  double min, max;

  display_.mtf().get_input_range(&min, &max);
  settings.setValue(QString("%1/imin").arg(prefix), min);
  settings.setValue(QString("%1/imax").arg(prefix), max);
  settings.setValue(QString("%1/cmap").arg(prefix), toString(display_.colormap()));
}


QLidarFrameVisualization::QLidarFrameVisualization()
{
  const auto add_display =
      [this](LIDAR_DISPLAY_TYPE display_type,
          double input_rage_min, double input_rage_max) -> QLidarDisplay*
          {
            QLidarDisplay * display = new QLidarDisplay(display_type, this);
            displays.emplace(display_type, display);
            display->mtf().set_input_range(input_rage_min, input_rage_max);
            display->load_parameters();
            return display;
          };


  add_display(LIDAR_DISPLAY_DEPTH, 0, 100);
  add_display(LIDAR_DISPLAY_INTENSITY, -0.25, 1.0);
  add_display(LIDAR_DISPLAY_HEIGHT, -3, 10);
  add_display(LIDAR_DISPLAY_DISTANCE, 0, 100);
  add_display(LIDAR_DISPLAY_ELEVATION, -30, 30);
  add_display(LIDAR_DISPLAY_AZIMUTH, 0, 360);
  add_display(LIDAR_DISPLAY_TIMESTAMP, 0, 1);
  add_display(LIDAR_DISPLAY_ALIASING, 0, 3);
  add_display(LIDAR_DISPLAY_LASERID, 0, 128);
  add_display(LIDAR_DISPLAY_DATABLOCK, 0, 12);
  add_display(LIDAR_DISPLAY_PACKET, 0, 200);
}

QLidarFrameVisualization * QLidarFrameVisualization::instance()
{
  static QLidarFrameVisualization instance_;
  return &instance_;
}


QLidarDisplay * QLidarFrameVisualization::get(LIDAR_DISPLAY_TYPE display_type)
{
  const auto pos = displays.find(display_type);
  return pos == displays.end() ?  nullptr : pos->second;
}

void QLidarFrameVisualization::set_current_frame(const c_lidar_frame::sptr & frame)
{
  for( const auto & p : displays ) {
    p.second->set_current_frame(frame);
  }
}

void QLidarFrameVisualization::set_azimuthal_resolution(double deg_per_pix)
{
  for( const auto & p : displays ) {
    p.second->set_azimuthal_resolution(deg_per_pix);
  }
}

void QLidarFrameVisualization::set_start_azimuth(double start_azimuth_in_degrees)
{
  for( const auto & p : displays ) {
    p.second->set_start_azimuth(start_azimuth_in_degrees);
  }
}

void QLidarFrameVisualization::set_num_lasers(int n)
{
  for( const auto & p : displays ) {
    p.second->set_num_lasers(n);
  }
}
