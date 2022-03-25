/*
 * c_mtf_display_function.cc
 *
 *  Created on: Mar 19, 2022
 *      Author: amyznikov
 */

#include "c_mtf_image_display_function.h"


void c_mtf_image_display_function::set_colormap(COLORMAP v)
{
  colormap_ = v;
}

COLORMAP c_mtf_image_display_function::colormap() const
{
  return colormap_;
}

c_pixinsight_mtf & c_mtf_image_display_function::mtf()
{
  return mtf_;
}

const c_pixinsight_mtf & c_mtf_image_display_function::mtf() const
{
  return mtf_;
}

bool c_mtf_image_display_function::create_display_image(const cv::Mat & src, const cv::Mat & srcmask, cv::Mat & dst, int ddepth) const
{
  mtf_.apply(src, dst, ddepth);

  if ( colormap_ != COLORMAP_NONE && ddepth == CV_8U && dst.channels() == 1 ) {
    apply_colormap(dst, dst, colormap_);
  }

  if ( !srcmask.empty() ) {
    dst.setTo(0, ~srcmask);
  }

  return true;
}

