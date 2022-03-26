/*
 * QLidarDisplay.h
 *
 *  Created on: Mar 19, 2022
 *      Author: amyznikov
 */

#ifndef __QLidarDisplayFunction_h__
#define __QLidarDisplayFunction_h__

#include <QtCore/QtCore>
#include <core/lidar/c_lidar_display.h>

class QLidarDisplay : public QObject
{
  Q_OBJECT;
public:
  typedef QLidarDisplay ThisClass;
  typedef QObject Base;

  QLidarDisplay(LIDAR_DISPLAY_TYPE display_type,
      QObject * parent = Q_NULLPTR);

  LIDAR_DISPLAY_TYPE type() const;

  void set_azimuthal_resolution(double degrees_per_pixel);
  double azimuthal_resolution() const;

  void set_start_azimuth(double start_azimuth_in_degrees);
  double start_azimuth() const;

  void set_num_lasers(int n);
  int num_lasers() const;

  void set_current_frame(c_lidar_frame::sptr frame);
  const c_lidar_frame::sptr & current_frame() const;

  void set_colormap(COLORMAP v);
  COLORMAP colormap() const;

  c_pixinsight_mtf & mtf();
  const c_pixinsight_mtf & mtf() const;

  void create_range_image(cv::OutputArray output_range_image,
      cv::OutputArray output_display_image,
      cv::OutputArray output_mask);

  void create_point_cloud(std::vector<cv::Vec3f> * positions,
      std::vector<cv::Vec3b> * colors);

  void create_input_histogramm(cv::OutputArray H,
      double * hmin, double * hmax);

  void create_output_histogramm(cv::OutputArray H,
      double * hmin, double * hmax);

  void compute_input_data_range(double * minval,
      double * maxval) const;

  void load_parameters();

  void save_paramters();

signals:
  void update();

protected:
  c_lidar_display display_;
  c_lidar_frame::sptr current_frame_;
};


class QLidarFrameVisualization:
    public QObject
{
  Q_OBJECT;
public:
  typedef QLidarDisplay ThisClass;
  typedef QObject Base;

  QLidarFrameVisualization();

  static QLidarFrameVisualization * instance();

  QLidarDisplay * get(LIDAR_DISPLAY_TYPE data);

  void set_current_frame(const c_lidar_frame::sptr & );
  void set_azimuthal_resolution(double deg_per_pix);
  void set_start_azimuth(double start_azimuth_in_degrees);
  void set_num_lasers(int n);


protected:
  std::map<LIDAR_DISPLAY_TYPE, QLidarDisplay *> displays;
};

inline QLidarFrameVisualization * lidarDisplays()
{
  return QLidarFrameVisualization::instance();
}

inline QLidarDisplay * getLidarDisplay(LIDAR_DISPLAY_TYPE display_type)
{
  return lidarDisplays()->get(display_type);
}

#endif /* __QLidarDisplayFunction_h__ */
