/*
 * QLidarConfigFileSettings.cc
 *
 *  Created on: Mar 24, 2022
 *      Author: amyznikov
 */

#include "QLidarConfigFileSettings.h"

QHDLSensorTypeToConfigFileMappingGroupBox::QHDLSensorTypeToConfigFileMappingGroupBox(QWidget * parent)
  : Base(parent)
{
  vbox_ = new QVBoxLayout(this);
  hbox_ = new QHBoxLayout();

  tooltip_label_ = new QLabel(this);
  tooltip_label_->setTextFormat(Qt::TextFormat::PlainText);
  tooltip_label_->setWordWrap(true);
  tooltip_label_->setText(""
      "This option will associate specific sensor type with specific lidar settings xml file. "
      "If not explicitly associated then the builtin (hard coded in application) default settings will used.");


  hbox_->addWidget(sensorType_ctl = new QEnumComboBox<HDLSensorType>(this), 1, Qt::AlignLeft);
  sensorType_ctl->setToolTip("Select sensor type to associate with config file");

  hbox_->addWidget(configFilePathName_ctl = new QBrowsePathCombo(this), 100);
  configFilePathName_ctl->setToolTip("Specify path and file name to lidar config file associated with selected sensor type");

  vbox_->addWidget(tooltip_label_);
  vbox_->addLayout(hbox_);

  connect(sensorType_ctl, &QEnumComboBoxBase::currentItemChanged,
      [this]() {
        configFilePathName_ctl->setCurrentPath(
            get_hdl_lidar_specification_config_file(
                sensorType_ctl->currentItem()).c_str(),
            false);
      });

  connect(configFilePathName_ctl, &QBrowsePathCombo::pathChanged,
      [this]() {

        set_hdl_lidar_specification_config_file(
            sensorType_ctl->currentItem(),
            configFilePathName_ctl->currentPath().toStdString());

        saveHDLSensorTypeToConfigFileMapping();
      });


  configFilePathName_ctl->setCurrentPath(
      get_hdl_lidar_specification_config_file(
          sensorType_ctl->currentItem()).c_str(),
      false);
}


QLidarConfigFileSettingWidget::QLidarConfigFileSettingWidget(QWidget * parent)
  : Base("QLidarConfigFilesSetting", parent)
{

  //  configFilePath_ctl = add_textbox("XML configs search path:");
  //  configFilePath_ctl->setText(".;~/.qlidarview;/usr/share/qlidarview");
  //  configFilePath_ctl->setToolTip("Specify semicolon-delimited lidar config xml file search path");

  sensorToConfigFileMapping_ctl = add_widget<QHDLSensorTypeToConfigFileMappingGroupBox>("");
  sensorToConfigFileMapping_ctl->setAlignment(Qt::AlignLeft);
  sensorToConfigFileMapping_ctl->setTitle("Specific sensor type to xml file mapping");
}


void loadHDLSensorTypeToConfigFileMapping()
{
  QSettings settings;

  const c_enum_member *sensor_type =
      members_of<HDLSensorType>();

  for( ; sensor_type->name; ++sensor_type ) {
    set_hdl_lidar_specification_config_file((HDLSensorType) sensor_type->value,
        settings.value(QString("HDLSensorTypeToConfigFileMapping/%1").arg(sensor_type->name)).toString().toStdString());
  }
}

void saveHDLSensorTypeToConfigFileMapping()
{
  QSettings settings;

  const c_enum_member *sensor_type =
      members_of<HDLSensorType>();

  for( ; sensor_type->name; ++sensor_type ) {
    settings.setValue(QString("HDLSensorTypeToConfigFileMapping/%1").arg(sensor_type->name),
        get_hdl_lidar_specification_config_file((HDLSensorType) sensor_type->value).c_str());
  }
}

