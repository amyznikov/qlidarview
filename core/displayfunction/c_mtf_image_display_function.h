/*
 * c_mtf_display_function.h
 *
 *  Created on: Mar 19, 2022
 *      Author: amyznikov
 */

#ifndef __c_mtf_display_function_h__
#define __c_mtf_display_function_h__

#include "c_image_display_function.h"
#include <core/mtf/c_pixinsight_mtf.h>
#include <core/proc/colormap.h>

class c_mtf_image_display_function:
    public c_image_display_function
{
public:
  void set_colormap(COLORMAP v);
  COLORMAP colormap() const;

  c_pixinsight_mtf & mtf();
  const c_pixinsight_mtf & mtf() const;

  bool create_display_image(const cv::Mat & src, const cv::Mat & srcmask, cv::Mat & dst, int ddepth) const override;

protected:
  c_pixinsight_mtf mtf_;
  COLORMAP colormap_ = COLORMAP_NONE;
};

#endif /* __c_mtf_display_function_h__ */
