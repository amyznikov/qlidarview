/*
 * QAppGlobalOptions.h
 *
 *  Created on: Mar 24, 2022
 *      Author: amyznikov
 */

#ifndef __QAppGlobalOptions_h__
#define __QAppGlobalOptions_h__

#include <QtWidgets/QtWidgets>
#include <gui/qlidar/QLidarConfigFileSettings.h>


namespace qlidarview {

class QAppGlobalOptionsWidgets :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QAppGlobalOptionsWidgets ThisClass;
  typedef QWidget Base;

  QAppGlobalOptionsWidgets(QWidget * parent = Q_NULLPTR);

protected:
  QVBoxLayout * layout_ = Q_NULLPTR;
  QTabWidget * tabWidget_ = Q_NULLPTR;
  QLidarConfigFileSettingWidget * lidarConfigFileSettings_ctl = Q_NULLPTR;
};


class QAppGlobalOptionsDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QAppGlobalOptionsDialogBox ThisClass;
  typedef QDialog Base;

  QAppGlobalOptionsDialogBox(QWidget * parent = Q_NULLPTR);

signals:
  void visibilityChanged(bool visible);

protected:
protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
protected:
  QAppGlobalOptionsWidgets * optionsWidget_ = Q_NULLPTR;
  QPushButton * closeCtl_ = Q_NULLPTR;

  QSize lastWidnowSize_;
  QPoint lastWidnowPos_;
};

} // namespace qlidarview

#endif /* __QAppGlobalOptions_h__ */
