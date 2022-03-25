/*
 * QAppGlobalOptions.cc
 *
 *  Created on: Mar 24, 2022
 *      Author: amyznikov
 */
#include "QAppGlobalOptions.h"



#define ICON_settings       "settings"

static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/gui/icons/%1").arg(name));
}

static void init_resources()
{
  Q_INIT_RESOURCE(gui_resources);
}

namespace qlidarview {


QAppGlobalOptionsWidgets::QAppGlobalOptionsWidgets(QWidget * parent) :
    Base(parent)
{
  init_resources();

  layout_ = new QVBoxLayout(this);
  tabWidget_ = new QTabWidget(this);
  tabWidget_->addTab(lidarConfigFileSettings_ctl = new QLidarConfigFileSettingWidget(this), "Lidar config files");
  layout_->addWidget(tabWidget_);
}


QAppGlobalOptionsDialogBox::QAppGlobalOptionsDialogBox(QWidget * parent) :
    Base(parent)
{
  QVBoxLayout * vbox;
  QHBoxLayout * hbox;

  init_resources();

  setWindowIcon(getIcon(ICON_settings));
  setWindowTitle("Configure program settings");

  vbox = new QVBoxLayout(this);
  hbox = new QHBoxLayout();

  hbox->addWidget(closeCtl_ = new QPushButton("Close", this), 0, Qt::AlignRight);
  vbox->addWidget(optionsWidget_ = new QAppGlobalOptionsWidgets(this));
  vbox->addLayout(hbox);

  connect(closeCtl_, &QPushButton::clicked,
      this, &QDialog::accept);
}

void QAppGlobalOptionsDialogBox::showEvent(QShowEvent *e)
{
  if ( !lastWidnowSize_.isEmpty() ) {
    Base::move(lastWidnowPos_);
    Base::resize(lastWidnowSize_);
  }

  Base::showEvent(e);
  emit visibilityChanged(isVisible());
}

void QAppGlobalOptionsDialogBox::hideEvent(QHideEvent *e)
{
  lastWidnowSize_ = this->size();
  lastWidnowPos_ = this->pos();
  Base::hideEvent(e);
  emit visibilityChanged(isVisible());
}


} // namespace qlidarview
