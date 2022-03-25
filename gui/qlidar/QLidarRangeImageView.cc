/*
 * QRangeImageView.cc
 *
 *  Created on: Mar 17, 2022
 *      Author: amyznikov
 */

#include "QLidarRangeImageView.h"
#include <core/ssprintf.h>
#include <core/debug.h>

QLidarRangeImageView::QLidarRangeImageView(QWidget * parent) :
    Base(parent)
{
  QSettings settings;

  const LIDAR_DISPLAY_TYPE displayType =
      fromString(settings.value("RangeImageLidarDisplay").toString().toStdString(),
          LIDAR_DISPLAY_DEPTH);

  setCurrentLidarDisplay(getLidarDisplay(displayType));
}

void QLidarRangeImageView::setCurrentLidarDisplay(QLidarDisplay * display)
{
  if( currentLidarDisplay_ ) {
    disconnect(currentLidarDisplay_, &QLidarDisplay::update,
        this, &ThisClass::updateDisplay);
  }

  if ( (currentLidarDisplay_ = display) ) {

    connect(currentLidarDisplay_, &QLidarDisplay::update,
        this, &ThisClass::updateDisplay);

    QSettings settings;
    settings.setValue("RangeImageLidarDisplay",
        toString(currentLidarDisplay_->type()));
  }
}


void QLidarRangeImageView::updateDisplay()
{
  if( currentLidarDisplay_ ) {
    currentImageData_.release();
    currentLidarDisplay_->create_range_image(currentImage_,
        displayImage_, currentMask_);
  }
  else {
    currentImage_.release();
    currentMask_.release();
    currentImageData_.release();
    displayImage_.release();
    qimage_ = QImage();
  }

  showCurrentDisplayImage();

  emit currentImageChanged();
  emit currentDisplayImageChanged();
}


QLidarRangeImageViewDock::QLidarRangeImageViewDock(const QString & title, QWidget * parent, QWidget * view, Qt::WindowFlags flags) :
  Base(title, parent, view, flags)
{
  //QLidarRangeImageView
}

void QLidarRangeImageViewDock::setCurrentLidarDisplay(QLidarDisplay * display)
{
  QLidarRangeImageView *view = imageView();
  if( view ) {
    view->setCurrentLidarDisplay(display);
    titleBar()->titleLabel()->setText(display ? toString(display->type()) : "Range Image");
  }
}

QLidarDisplay * QLidarRangeImageViewDock::currentLidarDisplay() const
{
  QLidarRangeImageView *view = imageView();
  return view ? view->currentLidarDisplay() : nullptr;
}

QLidarRangeImageView* QLidarRangeImageViewDock::imageView() const
{
  return dynamic_cast<QLidarRangeImageView*>(Base::widget());
}

QLidarRangeImageViewDock* addLidarRangeImageDock(QMainWindow * parent, Qt::DockWidgetArea area,
    const QString & dockName, const QString & title, QMenu * viewMenu)
{
  QLidarRangeImageView *view = new QLidarRangeImageView();

  QLidarRangeImageViewDock *dock =
      addDock<QLidarRangeImageViewDock>(parent,
          area,
          dockName,
          title,
          view,
          viewMenu);

  QObject::connect(view, &QLidarRangeImageView::currentImageChanged,
      [dock, view]() {

        const cv::Size imageSize = view->currentImage().size();
        const QLidarDisplay * display = view->currentLidarDisplay();

        if ( !display ) {
          dock->titleBar()->titleLabel()->setText(QString("RANGE IMAGE: %1x%2").
              arg(imageSize.width).
              arg(imageSize.height));
        }
        else {
          dock->titleBar()->titleLabel()->setText(QString("%1: %2x%3").
              arg(toString(display->type())).
              arg(imageSize.width).
              arg(imageSize.height));
        }
      });

  return dock;
}


