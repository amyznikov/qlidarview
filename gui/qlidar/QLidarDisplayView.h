/*
 * QLidarDisplayView.h
 *
 *  Created on: Mar 20, 2022
 *      Author: amyznikov
 */

#ifndef __QLidarDisplayView_h__
#define __QLidarDisplayView_h__

#include "QLidarDisplay.h"

class QLidarDisplayView
{
public:

  virtual ~QLidarDisplayView() = default;

  virtual void setCurrentLidarDisplay(QLidarDisplay * display);
  QLidarDisplay * currentLidarDisplay() const;

protected:
  QLidarDisplay * currentLidarDisplay_ = Q_NULLPTR;
};

#endif /* __QLidarDisplayView_h__ */
