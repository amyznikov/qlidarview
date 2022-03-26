/*
 * c_lidar_display.h
 *
 *  Created on: Mar 19, 2022
 *      Author: amyznikov
 */

#ifndef __c_lidar_display_h__
#define __c_lidar_display_h__

#include "c_lidar_frame.h"
#include <core/mtf/c_pixinsight_mtf.h>
#include <core/proc/colormap.h>


enum LIDAR_DISPLAY_TYPE {
  LIDAR_DISPLAY_DEPTH = 0,
  LIDAR_DISPLAY_DISTANCE,
  LIDAR_DISPLAY_INTENSITY,
  LIDAR_DISPLAY_HEIGHT,
  LIDAR_DISPLAY_ELEVATION,
  LIDAR_DISPLAY_AZIMUTH,
  LIDAR_DISPLAY_ALIASING,
  LIDAR_DISPLAY_LASERID,
  LIDAR_DISPLAY_DATABLOCK,
  LIDAR_DISPLAY_PACKET,
  LIDAR_DISPLAY_TIMESTAMP,
};

class c_lidar_display
{
public:
  c_lidar_display();

  void set_display_type(LIDAR_DISPLAY_TYPE v);
  LIDAR_DISPLAY_TYPE display_type() const;

  void set_azimuthal_resolution(double degrees_per_pixel);
  double azimuthal_resolution() const;

  void set_start_azimuth(double start_azimuth_in_degrees);
  double start_azimuth() const;

  void set_num_lasers(int n);
  int num_lasers() const;

  void set_colormap(COLORMAP v);
  COLORMAP colormap() const;

  c_pixinsight_mtf & mtf();
  const c_pixinsight_mtf & mtf() const;

  void compute_input_data_range(const c_lidar_frame * frame,
      double * minval, double * maxval) const;

  void create_input_histogramm(const c_lidar_frame * frame,
      cv::OutputArray H,
      double * hmin, double * hmax);

  void create_output_histogramm(const c_lidar_frame * frame,
      cv::OutputArray H,
      double * hmin, double * hmax);

  void create_range_image(const c_lidar_frame * frame,
      cv::OutputArray output_range_image,
      cv::OutputArray output_display_image,
      cv::OutputArray output_mask) const;

  void create_point_cloud(const c_lidar_frame * frame,
      std::vector<cv::Vec3f> * positions,
      std::vector<cv::Vec3b> * colors);



protected:
  void create_aliasing_image_(const c_lidar_frame * frame,
      cv::OutputArray output_range_image,
      cv::OutputArray output_display_image) const;

  void create_histogramm_(const c_lidar_frame * frame,
      double hmin, double hmax, int nbins, int channels,
      cv::OutputArray H,
      const c_pixinsight_mtf * mtf = nullptr) const;



protected:
  LIDAR_DISPLAY_TYPE display_type_ = LIDAR_DISPLAY_DEPTH; // use set_display_type()
  double start_azimuth_ = 180; // [deg]
  double azimuthal_resolution_ = 0.2; // deg / pix, use set_azimuthal_resolution()
  int num_lasers_ = 32; // just default, use set_num_lasers()

  c_pixinsight_mtf mtf_;
  COLORMAP colormap_ = COLORMAP_NONE;
  cv::Mat3b lut_;
};

#endif /* __c_lidar_display_h__ */
