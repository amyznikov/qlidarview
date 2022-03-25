/*
 * QLidarDisplayView.cc
 *
 *  Created on: Mar 20, 2022
 *      Author: amyznikov
 */

#include "QLidarDisplayView.h"

void QLidarDisplayView::setCurrentLidarDisplay(QLidarDisplay * display)
{
  currentLidarDisplay_ = display;
}

QLidarDisplay * QLidarDisplayView::currentLidarDisplay() const
{
  return currentLidarDisplay_;
}
