/*
 * QCloudViewSettings.h
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCloudViewSettings_h__
#define __QCloudViewSettings_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/lidar/c_hdl_lidar_specifcation.h>
#include "QLidarCloudView.h"


class QLidarCloudViewSettings
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QLidarCloudViewSettings ThisClass;
  typedef QSettingsWidget Base;

  QLidarCloudViewSettings(QWidget * parent = Q_NULLPTR);

  void setCloudViewer(QLidarCloudView * v);
  QLidarCloudView * cloudViewer() const;

  void showCurrentSensorInfo(HDLSensorType sensor_type, HDLReturnMode return_mode = HDLReturnMode_unknown);

  // these are temporary controls until I add dedicated settings widgets
  HDLFramingMode hdlFramingMode() const;
  double hdlFrameSeamAzimuth() const;
  double lidarDsplayAzimuthalResolution() const;
  double lidarDsplayStartAzimuth() const;

signals:
  void framingModeChanged();
  void hdlFrameSeamAzimuthChanged(double v);
  void lidarDisplayAzimuthalResolutionChanged(double v);
  void lidarDisplayStartAzimuthChanged(double v);

protected:
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected:
  QLidarCloudView * cloudViewer_ = Q_NULLPTR;
  QLabel * sensorType_ctl = Q_NULLPTR;
  QNumberEditBox * sceneRadius_ctl = Q_NULLPTR;
  QNumberEditBox * sceneOrigin_ctl = Q_NULLPTR;
  QNumberEditBox * pointSize_ctl = Q_NULLPTR;
  QEnumComboBox<HDLFramingMode> * framingMode_crl = Q_NULLPTR;
  QNumberEditBox * hdlFrameSeamAzimuth_ctl = Q_NULLPTR;
  QNumberEditBox * lidarDsplayAzimuthalResolution_ctl = Q_NULLPTR;
  QNumberEditBox * lidarDsplayStartAzimuth_ctl = Q_NULLPTR;
};

class QCloudViewSettingsDialogBox
    : public QDialog
{
  Q_OBJECT;
public:
  typedef QCloudViewSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QCloudViewSettingsDialogBox(QWidget * parent = Q_NULLPTR);

  void setCloudViewer(QLidarCloudView * v);
  QLidarCloudView * cloudViewer() const;

signals:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
protected:
  QVBoxLayout * vbox_ = Q_NULLPTR;
  QLidarCloudViewSettings * cloudViewSettings_ = Q_NULLPTR;
  QSize lastWidnowSize_;
  QPoint lastWidnowPos_;

};

#endif /* __QCloudViewSettings_h__ */
