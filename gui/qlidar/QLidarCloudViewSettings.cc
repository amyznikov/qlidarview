/*
 * QCloudViewSettings.cc
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#include "QLidarCloudViewSettings.h"
#include <core/debug.h>



QLidarCloudViewSettings::QLidarCloudViewSettings(QWidget * parent)
  : Base("QCloudViewSettings", parent)
{

  //sensorType_ctl = add_widget<QLabel>("");

  sceneRadius_ctl = add_numeric_box<double>("sceneRadius",
      [this](double v) {
        if ( cloudViewer_ && v > 0 ) {
          cloudViewer_->setSceneRadius(v);
          save_parameter(PREFIX, "sceneRadius", v);
        }
      });

  sceneOrigin_ctl = add_numeric_box<QGLVector>("sceneOrigin",
      [this](const QGLVector & v) {
        if ( cloudViewer_ && v != cloudViewer_->sceneOrigin() ) {
          cloudViewer_->setSceneOrigin(v);
          save_parameter(PREFIX, "SceneOrigin", v);
        }
      });

  pointSize_ctl = add_numeric_box<double>("pointSize",
      [this](double v) {
        if ( cloudViewer_ && v > 0 ) {
          cloudViewer_->setPointSize(v);
          save_parameter(PREFIX, "pointSize", v);
        }
      });

  framingMode_crl = add_enum_combobox<HDLFramingMode>("Framing mode",
      [this](HDLFramingMode v) {
        save_parameter(PREFIX, "framingMode", v);
        emit framingModeChanged();
      });

  lidarDsplayAzimuthalResolution_ctl = add_numeric_box<double>("range image resolution [deg/pix]",
      [this](double v) {
        if ( v > 0 ) {
          save_parameter(PREFIX, "lidarDsplayAzimuthalResolution", v);
          emit lidarDsplayAzimuthalResolutionChanged(v);
        }
      });
}

void QLidarCloudViewSettings::setCloudViewer(QLidarCloudView * v)
{
  cloudViewer_ = v;
  updateControls();
}

QLidarCloudView * QLidarCloudViewSettings::cloudViewer() const
{
  return cloudViewer_;
}

void QLidarCloudViewSettings::showCurrentSensorInfo(HDLSensorType sensor_type, HDLReturnMode return_mode)
{
  if ( sensor_type == HDLSensor_unknown ) {
    if ( sensorType_ctl ) {
      form->removeRow(sensorType_ctl);
      sensorType_ctl = Q_NULLPTR;
    }
  }
  else {
    if ( !sensorType_ctl ) {
      form->insertRow(0, sensorType_ctl = new QLabel());
      sensorType_ctl->setTextFormat(Qt::RichText);
      sensorType_ctl->setTextInteractionFlags(Qt::TextSelectableByMouse|Qt::TextSelectableByKeyboard);
    }
    sensorType_ctl->setText(QString("<strong>%1</strong><br>%2").
        arg(toString(sensor_type)).
        arg(toString(return_mode)));
  }
}

HDLFramingMode QLidarCloudViewSettings::hdlFramingMode() const
{
  return framingMode_crl->currentItem();
}

double QLidarCloudViewSettings::lidarDsplayAzimuthalResolution() const
{
  double value = -1;
  fromString(lidarDsplayAzimuthalResolution_ctl->text(), &value);
  return value;
}


void QLidarCloudViewSettings::onload(QSettings & settings)
{
  if ( cloudViewer_ ) {

    double sceneRadius = cloudViewer_->sceneRadius();
    if ( load_parameter(settings, PREFIX, "sceneRadius", &sceneRadius) ) {
      CF_DEBUG("cloudViewer_->setSceneRadius(sceneRadius=%g)", sceneRadius);
      cloudViewer_->setSceneRadius(sceneRadius);
    }

    QGLVector sceneOrigin = cloudViewer_->sceneOrigin();
    if ( load_parameter(settings, PREFIX, "sceneOrigin", &sceneOrigin) ) {
      cloudViewer_->setSceneOrigin(sceneOrigin);
    }

    double pointSize = cloudViewer_->pointSize();
    if ( load_parameter(settings, PREFIX, "pointSize", &pointSize) ) {
      cloudViewer_->setPointSize(pointSize);
    }

    QGLVector sceneCenter = cloudViewer_->sceneCenter();
    if ( load_parameter(settings, PREFIX, "sceneCenter", &sceneCenter) ) {
      cloudViewer_->setSceneCenter(sceneCenter);
    }

    HDLFramingMode v = hdlFramingMode();
    if ( load_parameter(settings, PREFIX, "framingMode", &v) ) {
      framingMode_crl->setCurrentItem(v);
    }

    double lidarDsplayAzimuthalResolution = 0.2;
    load_parameter(settings, PREFIX, "lidarDsplayAzimuthalResolution", &lidarDsplayAzimuthalResolution);
    if ( lidarDsplayAzimuthalResolution <= 0 ) {
      lidarDsplayAzimuthalResolution = 0.2;
    }
    lidarDsplayAzimuthalResolution_ctl->setValue(lidarDsplayAzimuthalResolution);
  }

}

void QLidarCloudViewSettings::onupdatecontrols()
{
  //cloudsSettings_ctl->setCloudViewer(cloudViewer_);

  if ( !cloudViewer_ ) {
    setEnabled(false);
  }
  else {

    sceneRadius_ctl->setValue(cloudViewer_->sceneRadius());
    sceneOrigin_ctl->setValue(toString(cloudViewer_->sceneOrigin()));
    pointSize_ctl->setValue(cloudViewer_->pointSize());

    setEnabled(true);
  }
}



///////////////////////////////////////////////////////////////////////////////
QCloudViewSettingsDialogBox::QCloudViewSettingsDialogBox(QWidget * parent)
  : Base(parent)
{
  vbox_ = new QVBoxLayout(this);
  vbox_->addWidget(cloudViewSettings_ = new QLidarCloudViewSettings(this));
}

void QCloudViewSettingsDialogBox::setCloudViewer(QLidarCloudView * v)
{
  cloudViewSettings_->setCloudViewer(v);
}

QLidarCloudView * QCloudViewSettingsDialogBox::cloudViewer() const
{
  return cloudViewSettings_->cloudViewer();
}

void QCloudViewSettingsDialogBox::showEvent(QShowEvent *e)
{
  Base::showEvent(e);
  emit visibilityChanged(isVisible());
}

void QCloudViewSettingsDialogBox::hideEvent(QHideEvent *e)
{
  Base::hideEvent(e);
  emit visibilityChanged(isVisible());
}

