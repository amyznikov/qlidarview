/*
 * QLidarDisplaySettings.h
 *
 *  Created on: Mar 19, 2022
 *      Author: amyznikov
 */

#ifndef __QLidarDisplaySettings_h__
#define __QLidarDisplaySettings_h__

#include <QtWidgets/QtWidgets>
#include <gui/qmtfcontrols/QHistogramView.h>
#include <gui/qmtfcontrols/QMtfSlider.h>
#include <gui/widgets/QLineEditBox.h>
#include <gui/widgets/QEnumComboBox.h>
#include "QLidarDisplayView.h"

class QLidarDisplaySettingsWidget:
    public QWidget
{
  Q_OBJECT;
public:
  typedef QLidarDisplaySettingsWidget ThisClass;
  typedef QWidget Base;

  QLidarDisplaySettingsWidget(QWidget * parent = Q_NULLPTR);

  void setLidarDisplayView(QLidarDisplayView * displayView) ;
  QLidarDisplayView * lidarDisplayView() const;

  bool isAutoMtfActionEnabled() const;
  bool updatingControls() const;
  void setUpdatingControls(bool v) ;

protected slots:
  void updateControls();
  void updateHistogramLevels();
  void onChartTypeSelectorClicked();
  void onResetMtfClicked();
  void onAutoMtfCtrlClicked();
  void onColormapCtlClicked();
  void onDisplayTypeCurrentItemChanged();
  void onInputDataRangeChanged();

protected:
  void resizeEvent(QResizeEvent *event) override;
  void findAutoHistogramClips();
  void findAutoMidtonesBalance();
  void updateColormapStrip();
  void updateColormapPixmap();

protected:
  QLidarDisplayView * lidarDisplayView_ = Q_NULLPTR;

  QVBoxLayout * vbox_ = Q_NULLPTR;
  QToolBar * topToolbar_ = Q_NULLPTR;
  QEnumComboBox<LIDAR_DISPLAY_TYPE> * displayType_ctl = Q_NULLPTR;
  QNumberEditBox * inputDataRange_ctl = Q_NULLPTR;
  QToolButton * colormap_ctl = Q_NULLPTR;

  QAction * resetMtfAction_ = Q_NULLPTR;
  QToolButton * autoMtf_ctl = Q_NULLPTR;
  QMenu autoMtfMenu;

  QAction * logScaleSelectionAction_ = Q_NULLPTR;
  QToolButton * chartTypeSelectorButton_ = Q_NULLPTR;

  QHistogramView * levelsView_ = Q_NULLPTR;
  QMtfSlider * mtfSlider_ = Q_NULLPTR;
  QLabel * colormap_strip_ctl = Q_NULLPTR;
  QPixmap colormap_pixmap_;

  QToolBar * bottomToolbar_ = Q_NULLPTR;

  enum AutoMtfAction {
    AutoMtfAction_AutoClip = 0,
    AutoMtfAction_AutoMtf = 1,
  } selectedAutoMtfAction_ = AutoMtfAction_AutoClip;

  enum SPINID {
    SPIN_SHADOWS = 0,
    SPIN_MIDTONES = 1,
    SPIN_HIGHLIGHTS = 2,
  };

  QDoubleSpinBox * spins[3] = {
      Q_NULLPTR };

  //  cv::Mat inputImage_;
  //  cv::Mat inputMask_;

  bool updatingControls_ = false;
};


class QLidarDisplaySettings:
    public QDialog
{
  Q_OBJECT;
public:
  typedef QLidarDisplaySettings ThisClass;
  typedef QDialog Base;

  QLidarDisplaySettings(QWidget * parent = Q_NULLPTR);

  void setLidarDisplayView(QLidarDisplayView * displayView) ;
  QLidarDisplayView * lidarDisplayView() const;

signals:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected:
  QVBoxLayout * vbox_ = Q_NULLPTR;
  QLidarDisplaySettingsWidget * widget = Q_NULLPTR;
  QSize lastWidnowSize_;
  QPoint lastWidnowPos_;
};

#endif /* __QLidarDisplaySettings_h__ */
