/*
 * MainWindow.h
 *
 *  Created on:
 *      Author: amyznikov
 */
#pragma once
#ifndef __qlidarview_main_window_h__
#define __qlidarview_main_window_h__

#include <QtWidgets/QtWidgets>
#include <gui/qcustomdock/QCustomDock.h>
#include <core/lidar/c_lidar_frame_loader.h>
#include <gui/qlidar/QLidarCloudView.h>
#include <gui/qlidar/QLidarCloudViewSettings.h>
#include <gui/qlidar/QLidarRangeImageView.h>
#include <gui/qlidar/QLidarDisplaySettings.h>
#include <gui/qimagesave/QImageSaveOptions.h>
#include <gui/widgets/QErrorMsgBox.h>
#include "QAppGlobalOptions.h"

namespace qlidarview {
///////////////////////////////////////////////////////////////////////////////


class MainWindow
    : public QMainWindow
{
  Q_OBJECT;
public:
  typedef MainWindow ThisClass;
  typedef QMainWindow Base;

  MainWindow();
  ~MainWindow();

private:
  void saveGeometry();
  void restoreGeometry();
  void saveState();
  void restoreState();
  void setupMainToolbar();
  void setupRangeImageView();
  void setupCloudViewSettingsDock();
  void createLidarDisplaySettingsControl();
  bool eventFilter(QObject *watched, QEvent *event) override;
  QLidarDisplayView * findActiveLidarDisplay();

public slots:
  void onOpenLidarFile();
  void openLidarFile(const QString & abspath);
  void setLidarDisplayAzimuthalResolution(double v);
  void onSaveCurrentRangeImageAs();
  void onSaveCurrentRangeDisplayImageAs();

private slots:
  void updateWindowTittle();
  void onFocusChanged(QWidget * old, QWidget * now);
  void onLidarDisplaySettingsMenuActionClicked(bool checked);
  void onShowGlobalOptionsMenuActionClicked(bool checked);

private:
  c_lidar_frame_loader::uptr lidar_frame_loader;

  QToolBar * mainToolbar = Q_NULLPTR;
  QStackedWidget * centralStackedWidget = Q_NULLPTR;
  QLidarCloudView * cloudViewer = Q_NULLPTR;
  QLidarCloudViewSettings * cloudViewSettings = Q_NULLPTR;
  QCustomDockWidget * cloudViewSettingsDock = Q_NULLPTR;
  QLidarRangeImageView * rangeImageView = Q_NULLPTR;
  QLidarRangeImageViewDock * rangeImageDock = Q_NULLPTR;
  QLidarDisplaySettings * lidarDisplaySettings = Q_NULLPTR;
  QLidarDisplayView * currentLidarDisplayView = Q_NULLPTR;
  QAppGlobalOptionsDialogBox * globalOptionsDlgBox = Q_NULLPTR;

  QMenu * fileMenu = Q_NULLPTR;
  QMenu * viewMenu = Q_NULLPTR;
  QMenu * editMenu = Q_NULLPTR;

  QAction * nextFrameAction = Q_NULLPTR;
  QAction * previousFrameAction = Q_NULLPTR;
  QAction * lidarDisplaySettingsMenuAction = Q_NULLPTR;
  QAction * showGlobalOptionsMenuAction = Q_NULLPTR;
  QAction * copyRangeImageToClipboardAction = Q_NULLPTR;
  QAction * copyPointCloutViewToClipboardAction = Q_NULLPTR;
  QAction * saveCurrentRangeImageAction = Q_NULLPTR;
  QAction * saveCurrentRangeDisplayImageAction = Q_NULLPTR;

};



///////////////////////////////////////////////////////////////////////////////
}  // namespace qlidarview
#endif /* __qlidarview_main_window_h__ */
