/*
 * MainWindow.cc
 *
 *  Created on: Feb 14, 2018
 *      Author: amyznikov
 */

#include "MainWindow.h"
#include <core/debug.h>

namespace qlidarview {
///////////////////////////////////////////////////////////////////////////////


#define ICON_reload       "reload"
#define ICON_prev         "prev"
#define ICON_next         "next"
#define ICON_like         "like"
#define ICON_dislike      "dislike"
#define ICON_close        "close"
#define ICON_histogram    "histogram"
#define ICON_marker_blue  "marker-blue"
#define ICON_reference    "reference"
#define ICON_settings      "settings"


#define ICON_copy         "copy"
#define ICON_delete       "delete"


static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/gui/icons/%1").arg(name));
}


MainWindow::MainWindow()
{
  QAction * action;

  ///////////////////////////////////
  // Setup central layout

  setWindowIcon(QIcon(":/icons/qlidarview.png"));
  updateWindowTittle();

  setCentralWidget(centralStackedWidget = new QStackedWidget(this));
  centralStackedWidget->addWidget(cloudViewer = new QLidarCloudView(this));

  ///////////////////////////////////
  // Setup main menu items

  menuBar()->setNativeMenuBar(false);
  fileMenu = menuBar()->addMenu("&File");
  editMenu = menuBar()->addMenu("&Edit");
  viewMenu = menuBar()->addMenu("&View");

  fileMenu->addAction(action = new QAction("Open lidar file ..."));
  connect(action, &QAction::triggered,
      this, &ThisClass::onOpenLidarFile);

  fileMenu->addSeparator();
  fileMenu->addAction(saveCurrentRangeImageAction = new QAction("Save range image as ..."));
  connect(saveCurrentRangeImageAction, &QAction::triggered,
      this, &ThisClass::onSaveCurrentRangeImageAs);

  fileMenu->addAction(saveCurrentRangeDisplayImageAction = new QAction("Save range display image as ..."));
  connect(saveCurrentRangeDisplayImageAction, &QAction::triggered,
      this, &ThisClass::onSaveCurrentRangeDisplayImageAs);

  fileMenu->addSeparator();
  fileMenu->addAction(action = new QAction("Quit"));
  connect(action, &QAction::triggered, []() {
    QApplication::quit();
  });

  editMenu->addAction(copyPointCloutViewToClipboardAction = new QAction("Copy cloud view snapshot"));
  connect(copyPointCloutViewToClipboardAction, &QAction::triggered,
      [this]() {
      if ( cloudViewer ) {
        cloudViewer->snapshotToClipboard();
      }
  });

  ///////////////////////////////////
  // Setup main application toolBar
  setupMainToolbar();


  ///////////////////////////////////
  // Setup docking views
  setDockOptions(AnimatedDocks | AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
  setCorner( Qt::TopLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::TopRightCorner, Qt::RightDockWidgetArea );
  setCorner( Qt::BottomLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::BottomRightCorner, Qt::BottomDockWidgetArea );

  setupRangeImageView();
  setupCloudViewSettingsDock();


  ///////////////////////////////////
  // Restore saved state
  restoreGeometry();
  restoreState();

  ///////////////////////////////////
  // Setup global app options

  loadHDLSensorTypeToConfigFileMapping();

  ///////////////////////////////////
  // Setup global event filters

  QApplication::instance()->
      installEventFilter(this);

  connect(qApp, &QApplication::focusChanged,
      this, &ThisClass::onFocusChanged);




  ///////////////////////////////////
  // Finish initialization
  statusBar()->showMessage("Ready");
}

MainWindow::~MainWindow()
{
  saveState();
  saveGeometry();
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
  if ( event->type() == QEvent::Wheel) {
    const QComboBox * combo = dynamic_cast<const QComboBox*>(watched);
    if ( combo && !combo->isEditable() ) {
      return true;
    }
  }

  return Base::eventFilter(watched, event);
}

void MainWindow::updateWindowTittle()
{
  if ( !lidar_frame_loader ) {
    setWindowTitle("qlidarview");
  }
  else {
    setWindowTitle(QString("qlidarview: %1").arg(lidar_frame_loader->filename().c_str()));
  }
}

void MainWindow::saveGeometry()
{
  QSettings settings;
  settings.setValue("MainWindow/Geometry", Base::saveGeometry());
  settings.setValue("MainWindow/State", Base::saveState());
}

void MainWindow::restoreGeometry()
{
  QSettings settings;
  Base::restoreGeometry(settings.value("MainWindow/Geometry").toByteArray());
  Base::restoreState(settings.value("MainWindow/State").toByteArray());
}

void MainWindow::saveState()
{
//  QSettings settings;
//  settings.setValue("fileSystemTree/absoluteFilePath",
//      fileSystemTreeDock->currentAbsoluteFilePath());
}

void MainWindow::restoreState()
{
//  QSettings settings;
//  fileSystemTreeDock->displayPath(settings.value(
//      "fileSystemTree/absoluteFilePath").toString());
}


void MainWindow::setupMainToolbar()
{
  mainToolbar = new QToolBar();
  mainToolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
  mainToolbar->setIconSize(QSize(16,16));
  menuBar()->setCornerWidget(mainToolbar);

  mainToolbar->addAction(previousFrameAction = new QAction(getIcon(ICON_prev), "Previous frame"));
  previousFrameAction->setToolTip("Load previous frame (Ctrl+PageUp)\n"
      "Does not work with .pcap files in current implementation");
  //previousFrameAction->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageUp));
  previousFrameAction->setEnabled(false);
  // connect(previousFrameAction, &QAction::triggered, [this]() {
  //  thumbnailsView->selectPrevIcon();
  // });

  mainToolbar->addAction(nextFrameAction = new QAction(getIcon(ICON_next), "Next frame"));
  nextFrameAction->setToolTip("Load next frame (Ctrl+PageDown)");
  nextFrameAction->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageDown));
  nextFrameAction->setEnabled(false);
  connect(nextFrameAction, &QAction::triggered,
      [this]() {
        if ( lidar_frame_loader ) {

          c_lidar_frame::sptr frame =
              lidar_frame_loader->load_next_frame();

          lidarDisplays()->set_current_frame(frame);

          if ( !frame ) {
            QErrorMsgBox::show(this, "ERROR",
                "load_next_frame() fails");
          }
        }
      });



  mainToolbar->addAction(lidarDisplaySettingsMenuAction = new QAction(getIcon(ICON_histogram), "Display options..."));
  lidarDisplaySettingsMenuAction->setToolTip("Configure lidar display options");
  lidarDisplaySettingsMenuAction->setCheckable(true);
  lidarDisplaySettingsMenuAction->setChecked(false);
  connect(lidarDisplaySettingsMenuAction, &QAction::triggered, this, &ThisClass::onLidarDisplaySettingsMenuActionClicked);


  mainToolbar->addAction(showGlobalOptionsMenuAction = new QAction(getIcon(ICON_settings), "Options ..."));
  showGlobalOptionsMenuAction->setToolTip("Setup application global options");
  showGlobalOptionsMenuAction->setCheckable(true);
  showGlobalOptionsMenuAction->setChecked(false);
  viewMenu->addAction(showGlobalOptionsMenuAction);
  connect(showGlobalOptionsMenuAction, &QAction::triggered, this, &ThisClass::onShowGlobalOptionsMenuActionClicked);
}


void MainWindow::onOpenLidarFile()
{
  QSettings settings;

  static const QString lastOpenLidarFileSettingsKey =
      "lastOpenLidarFile";

  QString lastLidarFileName =
      settings.value(lastOpenLidarFileSettingsKey, "").toString();

  static const QString filter =
      "PCAP files (*.pcap) ;;\n"
      "VPCAP files (.vpcap)";


  QString selectedFileName =
      QFileDialog::getOpenFileName(this,
          "Open lidar file ...",
          lastLidarFileName,
          filter);

  if ( selectedFileName.isEmpty() ) {
    return;
  }

  settings.setValue(lastOpenLidarFileSettingsKey,
      selectedFileName);

  openLidarFile(selectedFileName);
}

void MainWindow::openLidarFile(const QString & abspath)
{
  try {

    cloudViewSettings->showCurrentSensorInfo(HDLSensor_unknown);
    lidarDisplays()->set_current_frame(nullptr);

    lidar_frame_loader =
        c_lidar_frame_loader::open_file(
            abspath.toStdString());


    if( !lidar_frame_loader ) {
      nextFrameAction->setEnabled(false);
      QErrorMsgBox::show(this, "ERROR",
          "c_lidar_frame_loader :: open_file() fails");
    }
    else {
      nextFrameAction->setEnabled(true);

      lidar_frame_loader->set_hdl_framing_mode(
          cloudViewSettings->hdlFramingMode());

      const c_lidar_frame::sptr first_frame =
          lidar_frame_loader->load_next_frame();

      lidarDisplays()->set_num_lasers(lidar_frame_loader->lidar_specification()->lasers.size());
      lidarDisplays()->set_current_frame(first_frame);

      cloudViewSettings->showCurrentSensorInfo(lidar_frame_loader->lidar_specification()->sensor,
          lidar_frame_loader->return_mode());

      if ( !first_frame ) {
        QErrorMsgBox::show(this, "ERROR",
            "load_next_frame() fails");
      }
    }
  }
  catch( const std::exception &e ) {
    nextFrameAction->setEnabled(false);
    QErrorMsgBox::show(this, "ERROR", e.what());
  }

  updateWindowTittle();
}

void MainWindow::createLidarDisplaySettingsControl()
{
  if( !lidarDisplaySettings ) {

    lidarDisplaySettings = new QLidarDisplaySettings(this);

    connect(lidarDisplaySettings, &QLidarDisplaySettings::visibilityChanged,
        [this](bool visible) {
          lidarDisplaySettingsMenuAction->setChecked(visible);
        });
  }
}

void MainWindow::setupRangeImageView()
{
  QShortcut * shortcut;

  rangeImageDock =
      addLidarRangeImageDock(this,
          Qt::BottomDockWidgetArea,
          "rangeImageViewDock",
          "Range Image",
          viewMenu);

  rangeImageView = rangeImageDock->imageView();


  shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_C), rangeImageView);
  connect(shortcut, &QShortcut::activated, copyRangeImageToClipboardAction, &QAction::trigger);


  shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_S), rangeImageView);
  connect(shortcut, &QShortcut::activated, saveCurrentRangeDisplayImageAction, &QAction::trigger);


  shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::SHIFT + Qt::Key_S), rangeImageView);
  connect(shortcut, &QShortcut::activated, saveCurrentRangeImageAction, &QAction::trigger);

  rangeImageView->addAction(copyRangeImageToClipboardAction = new QAction("Copy range image", rangeImageView));
  editMenu->addAction(copyRangeImageToClipboardAction);
  connect(copyRangeImageToClipboardAction, &QAction::triggered,
      [this]() {
        if ( rangeImageView->isVisible() ) {
          rangeImageView->copyDisplayImageToClipboard();
        }
      });



  connect(rangeImageDock, &QDockWidget::visibilityChanged,
      [this](bool visible) {

        copyRangeImageToClipboardAction->setEnabled(visible );
        saveCurrentRangeImageAction->setEnabled(visible);
        saveCurrentRangeDisplayImageAction->setEnabled(visible);

        if ( visible ) {
          rangeImageDock->imageView()->setFocus();
          if ( lidarDisplaySettings && lidarDisplaySettings->isVisible() ) {
            lidarDisplaySettings->setLidarDisplayView(rangeImageDock->imageView());
          }
        }
        else if ( currentLidarDisplayView == rangeImageDock->imageView() ) {

          currentLidarDisplayView = findActiveLidarDisplay();
          if ( !currentLidarDisplayView ) {
            lidarDisplaySettingsMenuAction->setEnabled(false);
          }

          if ( lidarDisplaySettings && lidarDisplaySettings->isVisible() ) {
            if ( currentLidarDisplayView ) {
              lidarDisplaySettings->setLidarDisplayView(currentLidarDisplayView);
            }
            else {
              lidarDisplaySettings->hide();
              lidarDisplaySettings->setLidarDisplayView(nullptr);
            }
          }
        }
      });

  connect(rangeImageView, &QLidarRangeImageView::onMouseMove,
      [this](QMouseEvent * e) {
        statusBar()->showMessage(rangeImageView->statusStringForPixel(e->pos()));
      });
}


void MainWindow::onLidarDisplaySettingsMenuActionClicked(bool checked)
{
  if( !checked ) {
    if( lidarDisplaySettings && lidarDisplaySettings->isVisible() ) {
      lidarDisplaySettings->hide();
    }
    return;
  }

  CF_DEBUG("currentLidarDisplay=%p", currentLidarDisplayView);

  if( !currentLidarDisplayView && !(currentLidarDisplayView = findActiveLidarDisplay()) ) {
    lidarDisplaySettingsMenuAction->setChecked(false);
    return;
  }


  createLidarDisplaySettingsControl();
  lidarDisplaySettings->setLidarDisplayView(currentLidarDisplayView);
  if( !lidarDisplaySettings->isVisible() ) {
    lidarDisplaySettings->show();
  }
}

QLidarDisplayView * MainWindow::findActiveLidarDisplay()
{
  static const auto inFocus =
      [](QWidget * obj, QWidget * focusWidget) -> bool {
        return obj && obj->isVisible() &&
            (focusWidget == obj || obj->isAncestorOf(focusWidget));
      };

  QWidget *currentFocusWidget = qApp->focusWidget();
  if( currentFocusWidget ) {
    if ( inFocus(rangeImageDock, currentFocusWidget) ) {
      return rangeImageDock->imageView();
    }
    if ( inFocus(cloudViewer, currentFocusWidget) ) {
      return cloudViewer;
    }
  }

  return nullptr;
}


void MainWindow::onFocusChanged(QWidget * old, QWidget * now)
{
  QLidarDisplayView *actvatedLIdarDisplayView =
      findActiveLidarDisplay();

  if( actvatedLIdarDisplayView ) {
    currentLidarDisplayView = actvatedLIdarDisplayView;
  }

  lidarDisplaySettingsMenuAction->setEnabled(currentLidarDisplayView != nullptr);

  if( currentLidarDisplayView && lidarDisplaySettings && lidarDisplaySettings->isVisible() ) {
    if( currentLidarDisplayView != lidarDisplaySettings->lidarDisplayView() ) {
      lidarDisplaySettings->setLidarDisplayView(currentLidarDisplayView);
    }
  }
}

void MainWindow::setupCloudViewSettingsDock()
{
  cloudViewSettingsDock =
      addCustomDock(this,
          Qt::LeftDockWidgetArea,
          "cloudViewSettingsDock",
          "cloudViewSettings",
          cloudViewSettings = new QLidarCloudViewSettings(this),
          viewMenu);

  cloudViewSettings->setCloudViewer(cloudViewer);
  cloudViewSettings->loadParameters();
  //cloudViewer->setSceneRadius(cloudViewSettings->)

  connect(cloudViewSettings, &QLidarCloudViewSettings::framingModeChanged,
      [this]() {
        if ( lidar_frame_loader ) {
          lidar_frame_loader->set_hdl_framing_mode(cloudViewSettings->hdlFramingMode());
        }
      });

  connect(cloudViewSettings, &QLidarCloudViewSettings::lidarDsplayAzimuthalResolutionChanged,
      this, &ThisClass::setLidarDisplayAzimuthalResolution);

  setLidarDisplayAzimuthalResolution(cloudViewSettings->lidarDsplayAzimuthalResolution());
}

void MainWindow::onShowGlobalOptionsMenuActionClicked(bool checked)
{
  if( !globalOptionsDlgBox ) {
    globalOptionsDlgBox = new QAppGlobalOptionsDialogBox(this);
    connect(globalOptionsDlgBox, &QAppGlobalOptionsDialogBox::visibilityChanged,
        [this](bool visible) {
          showGlobalOptionsMenuAction->setChecked(visible);
        });
  }

  globalOptionsDlgBox->setVisible(checked);
}

void MainWindow::setLidarDisplayAzimuthalResolution(double v)
{
  if ( v > 0  ) {
    lidarDisplays()->set_azimuthal_resolution(v);
  }
}


void MainWindow::onSaveCurrentRangeImageAs()
{
  if( rangeImageView->isVisible() ) {

    const cv::Mat &currentImage =
        rangeImageView->currentImage();

    if( !currentImage.empty() ) {

      const cv::Mat &currentMask =
          rangeImageView->currentMask();

      saveImageFileAs(this,
          currentImage,
          currentMask);
    }
  }
}

void MainWindow::onSaveCurrentRangeDisplayImageAs()
{
  if( rangeImageView->isVisible() ) {

    const cv::Mat &currentImage =
        rangeImageView->displayImage();

    if( !currentImage.empty() ) {

      const cv::Mat &currentMask =
          rangeImageView->currentMask();

      saveImageFileAs(this,
          currentImage,
          currentMask);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
}  // namespace qlidarview
