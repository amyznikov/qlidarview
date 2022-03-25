/*
 * c_image_display_function.h
 *
 *  Created on: Mar 19, 2022
 *      Author: amyznikov
 */

#ifndef __c_display_function_h__
#define __c_display_function_h__

#include <opencv2/opencv.hpp>

class c_image_display_function
{
public:
  virtual ~c_image_display_function() = default;
  virtual bool create_display_image(const cv::Mat & src, const cv::Mat & srcmask, cv::Mat & dst, int ddepth) const = 0;
};

#endif /* __c_display_function_h__ */
