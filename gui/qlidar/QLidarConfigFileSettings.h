/*
 * QLidarConfigFileSettings.h
 *
 *  Created on: Mar 24, 2022
 *      Author: amyznikov
 */

#ifndef ___QLidarConfigFilesSetting_h___
#define ___QLidarConfigFilesSetting_h___

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <core/lidar/c_hdl_lidar_specifcation.h>


class QHDLSensorTypeToConfigFileMappingGroupBox :
    public QGroupBox
{
  Q_OBJECT;
public:
  typedef QHDLSensorTypeToConfigFileMappingGroupBox ThisClass;
  typedef QGroupBox Base;

  QHDLSensorTypeToConfigFileMappingGroupBox(QWidget * parent = Q_NULLPTR);

protected:
  QVBoxLayout * vbox_ = Q_NULLPTR;
  QHBoxLayout * hbox_ = Q_NULLPTR;
  QLabel * tooltip_label_ = Q_NULLPTR;
  QEnumComboBox<HDLSensorType> * sensorType_ctl = Q_NULLPTR;
  QBrowsePathCombo * configFilePathName_ctl = Q_NULLPTR;
};

class QLidarConfigFileSettingWidget:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QLidarConfigFileSettingWidget ThisClass;
  typedef QSettingsWidget Base;

  QLidarConfigFileSettingWidget(QWidget * parent = Q_NULLPTR);

protected:
  //QLineEditBox * configFilePath_ctl = Q_NULLPTR;
  QHDLSensorTypeToConfigFileMappingGroupBox * sensorToConfigFileMapping_ctl = Q_NULLPTR;
};

void loadHDLSensorTypeToConfigFileMapping();
void saveHDLSensorTypeToConfigFileMapping();

#endif /* ___QLidarConfigFilesSetting_h___ */
