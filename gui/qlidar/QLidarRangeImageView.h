/*
 * QRangeImageView.h
 *
 *  Created on: Mar 17, 2022
 *      Author: amyznikov
 */

#ifndef __QRangeImageView_h__
#define __QRangeImageView_h__

#include <gui/qimageview/QImageViewer.h>
#include <gui/qcustomdock/QCustomDock.h>
#include "QLidarDisplayView.h"


class QLidarRangeImageView:
    public QImageViewer,
    public QLidarDisplayView
{
  Q_OBJECT;
public:
  typedef QLidarRangeImageView ThisClass;
  typedef QImageViewer Base;

  QLidarRangeImageView(QWidget * parent = Q_NULLPTR);

  void setCurrentLidarDisplay(QLidarDisplay * func) override;

protected slots:
  void updateDisplay() override;
};



class QLidarRangeImageViewDock:
    public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QLidarRangeImageViewDock ThisClass;
  typedef QCustomDockWidget Base;

  QLidarRangeImageViewDock(const QString & title,
      QWidget * parent = Q_NULLPTR,
      QWidget * view = Q_NULLPTR,
      Qt::WindowFlags flags = Qt::WindowFlags());

  void setCurrentLidarDisplay(QLidarDisplay * func);
  QLidarDisplay * currentLidarDisplay() const;

  QLidarRangeImageView * imageView() const;

protected:
  QLidarRangeImageView * imageView_ = Q_NULLPTR;
};


QLidarRangeImageViewDock * addLidarRangeImageDock(QMainWindow * parent, Qt::DockWidgetArea area,
    const QString & dockName, const QString & title, QMenu * viewMenu = Q_NULLPTR);


#endif /* __QRangeImageView_h__ */
