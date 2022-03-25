/*
 * c_point_cloud_display_function.h
 *
 *  Created on: Mar 19, 2022
 *      Author: amyznikov
 */

#ifndef __c_point_cloud_display_function_h__
#define __c_point_cloud_display_function_h__

#include <opencv2/opencv.hpp>

class c_point_cloud_display_function
{
public:
  virtual ~c_point_cloud_display_function() = default;
  virtual cv::Vec3b compute_point_color(const cv::Scalar & srcpointdata, int scrchanels) const = 0;
};

#endif /* __c_point_cloud_display_function_h__ */
