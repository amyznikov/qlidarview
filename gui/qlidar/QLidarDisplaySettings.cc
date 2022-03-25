/*
 * QLidarDisplaySettings.cc
 *
 *  Created on: Mar 19, 2022
 *      Author: amyznikov
 */

#include "QLidarDisplaySettings.h"
#include <gui/widgets/QWaitCursor.h>
#include <gui/widgets/settings.h>
#include <gui/qimageview/cv2qt.h>
#include <core/proc/pixtype.h>
#include <core/proc/minmax.h>
#include <core/proc/histogram.h>

#define ICON_histogram                "histogram"
#define ICON_histogram_linear_scale   "histogram-linear-scale2"
#define ICON_histogram_log_scale      "histogram-log-scale2"
#define ICON_histogram_automtf        "histogram-automtf"
#define ICON_reset                    "reset"
#define ICON_contrast                 "contrast"
#define ICON_bar_chart                "bar_chart"
#define ICON_line_chart               "line_chart"
#define ICON_colormap                 "colormap2"

static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qmtfcontrols/icons/%1").arg(name));
}

static QPixmap getPixmap(const QString & name)
{
  return QPixmap(QString(":/qmtfcontrols/icons/%1").arg(name));
}

static QIcon log_scale_icon;
static QIcon bar_chart_icon;
static QIcon line_chart_icon;
static QIcon auto_clip_icon;
static QIcon auto_mtf_icon;
static QIcon colormap_icon;

static const QIcon & selectChartTypeIcon(QHistogramView::ChartType chartType)
{
  switch ( chartType ) {
  case QHistogramView::ChartType_Lines :
    return line_chart_icon;
  case QHistogramView::ChartType_Bars :
    default :
    break;
  }
  return bar_chart_icon;
}

static void addStretch(QToolBar * toolbar)
{
  QWidget* empty = new QWidget();
  empty->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  toolbar->addWidget(empty);
}

static void intit_mtfcontrols_resources()
{
  Q_INIT_RESOURCE(qmtfcontrols_resources);

  if ( bar_chart_icon.isNull() ) {
    bar_chart_icon = getIcon(ICON_bar_chart);
  }
  if ( line_chart_icon.isNull() ) {
    line_chart_icon = getIcon(ICON_line_chart);
  }
  if ( log_scale_icon.isNull() ) {
    log_scale_icon.addPixmap(getPixmap(ICON_histogram_linear_scale), QIcon::Normal, QIcon::Off);
    log_scale_icon.addPixmap(getPixmap(ICON_histogram_log_scale), QIcon::Normal, QIcon::On);
  }
  if ( auto_clip_icon.isNull() ) {
    auto_clip_icon = getIcon(ICON_contrast);
  }
  if ( auto_mtf_icon.isNull() ) {
    auto_mtf_icon = getIcon(ICON_histogram_automtf);
  }
  if ( colormap_icon.isNull() ) {
    colormap_icon = getIcon(ICON_colormap);
  }

}

QLidarDisplaySettingsWidget::QLidarDisplaySettingsWidget(QWidget * parent) :
    Base(parent)
{
  intit_mtfcontrols_resources();

  vbox_ = new QVBoxLayout(this);

  topToolbar_ = new QToolBar(this);
  topToolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  topToolbar_->setIconSize(QSize(16, 16));


  displayType_ctl = new QEnumComboBox<LIDAR_DISPLAY_TYPE>(this);
  displayType_ctl->setToolTip("Select data to visualize");
  topToolbar_->addWidget(displayType_ctl);
  connect(displayType_ctl, &QEnumComboBoxBase::currentItemChanged,
      this, &ThisClass::onDisplayTypeCurrentItemChanged);


  inputDataRange_ctl = new QNumberEditBox(this);
  inputDataRange_ctl->setToolTip("Set input data range (min/max clips)");
  topToolbar_->addWidget(inputDataRange_ctl);
  connect(inputDataRange_ctl, &QNumberEditBox::textChanged,
      this, &ThisClass::onInputDataRangeChanged);


  resetMtfAction_ = topToolbar_->addAction(getIcon(ICON_reset), "Reset");
  resetMtfAction_->setToolTip("Reset MTF clipping and recompute input data range");
  connect(resetMtfAction_, &QAction::triggered,
      this, &ThisClass::onResetMtfClicked);


  //
  addStretch(topToolbar_);

  colormap_ctl = new QToolButton();
  colormap_ctl->setIcon(colormap_icon);
  colormap_ctl->setText("Colormap");
  colormap_ctl->setToolTip("Select colormap");
  topToolbar_->addWidget(colormap_ctl);
  connect(colormap_ctl, &QToolButton::clicked,
      this, &ThisClass::onColormapCtlClicked);


  chartTypeSelectorButton_ = new QToolButton();
  chartTypeSelectorButton_->setText("Chart type");
  chartTypeSelectorButton_->setToolTip("Select chart type");
  topToolbar_->addWidget(chartTypeSelectorButton_);
  connect(chartTypeSelectorButton_, &QToolButton::clicked,
      this, &ThisClass::onChartTypeSelectorClicked );




  //
  autoMtf_ctl = new QToolButton(this);
  autoMtf_ctl->setIconSize(QSize(16, 16));
  autoMtf_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  autoMtf_ctl->setCheckable(true);
  autoMtf_ctl->setText("Auto MTF");
  autoMtf_ctl->setToolTip("Auto MTF adjustment");
  topToolbar_->addWidget(autoMtf_ctl);
  connect(autoMtf_ctl, &QToolButton::clicked,
      this, &ThisClass::onAutoMtfCtrlClicked );

  autoMtfMenu.addAction(auto_clip_icon,
      "Auto clip",
      [this]() {
        selectedAutoMtfAction_ = AutoMtfAction_AutoClip;
        onAutoMtfCtrlClicked();
      });

  autoMtfMenu.addAction(auto_mtf_icon,
      "Auto Mtf",
      [this]() {
        selectedAutoMtfAction_ = AutoMtfAction_AutoMtf;
        onAutoMtfCtrlClicked();
      });

  autoMtf_ctl->setPopupMode(QToolButton::MenuButtonPopup);
  autoMtf_ctl->setMenu(&autoMtfMenu);

  logScaleSelectionAction_ = topToolbar_->addAction(log_scale_icon, "Log scale");
  logScaleSelectionAction_ ->setToolTip("Switch between linear / log scale");
  logScaleSelectionAction_->setCheckable(true);
  logScaleSelectionAction_->setChecked(false);

  // configure gistogram view
  levelsView_ = new QHistogramView(this);
  logScaleSelectionAction_->setChecked(levelsView_->logScale());
  chartTypeSelectorButton_->setIcon(selectChartTypeIcon(levelsView_->chartType()) );
  //displayChannel_ctl->setCurrentIndex(displayChannel_ctl->findData((int)levelsView_->displayChannel()));

  // configure mtf slider
  mtfSlider_ = new QMtfSlider(this);
  colormap_strip_ctl = new QLabel(this);
  //  colormap_strip_ctl->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);

  // configure toolbar2
  bottomToolbar_ = new QToolBar(this);
  bottomToolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  bottomToolbar_->setIconSize(QSize(16, 16));

  for ( int i = 0; i < 3; ++i ) {
    bottomToolbar_->addWidget(spins[i] = new QDoubleSpinBox());
    spins[i]->setKeyboardTracking(false);
    spins[i]->setRange(0, 1);
    spins[i]->setDecimals(3);
    spins[i]->setSingleStep(0.01);
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
    spins[i]->setStepType(QAbstractSpinBox::AdaptiveDecimalStepType);
#endif
    if ( i < 2 ) {
      addStretch(bottomToolbar_);
    }
  }

  // Combine layouts
  vbox_->addWidget(topToolbar_, 1);
  vbox_->addWidget(levelsView_, 1000);
  vbox_->addWidget(mtfSlider_, 1);
  vbox_->addWidget(colormap_strip_ctl, 1);
  vbox_->addWidget(bottomToolbar_, 1);

  // Configure event handles

  connect(logScaleSelectionAction_, &QAction::triggered,
      levelsView_, &QHistogramView::setLogScale);

  connect(mtfSlider_, &QMtfSlider::mtfChanged,
      [this]() {
        if ( lidarDisplayView_ && !updatingControls_ ) {

          QLidarDisplay * display =
              lidarDisplayView_->currentLidarDisplay();

          if ( display ) {

            c_pixinsight_mtf & mtf =
                display->mtf();

            const bool wasInUpdatingControls =
                updatingControls();

            setUpdatingControls(true);

            mtf.set_shadows(mtfSlider_->shadows());
            mtf.set_highlights(mtfSlider_->highlights());
            mtf.set_midtones(mtfSlider_->midtones());

            spins[SPIN_SHADOWS]->setValue(mtf.shadows());
            spins[SPIN_HIGHLIGHTS]->setValue(mtf.highlights());
            spins[SPIN_MIDTONES]->setValue(mtf.midtones());

            setUpdatingControls(wasInUpdatingControls);
            if ( !wasInUpdatingControls ) {
              updateHistogramLevels();
              emit display->update();
            }
          }
        }
      });

  connect(spins[SPIN_SHADOWS], static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {
        if ( lidarDisplayView_ && !updatingControls_ ) {

          QLidarDisplay * display =
              lidarDisplayView_->currentLidarDisplay();

          if ( display ) {

            c_pixinsight_mtf & mtf =
                display->mtf();

            if ( v != mtf.shadows() ) {

              const bool wasInUpdatingControls =
              updatingControls();

              setUpdatingControls(true);

              mtf.set_shadows(v);
              mtfSlider_->setShadows(mtf.shadows());

              setUpdatingControls(wasInUpdatingControls);
              if ( !wasInUpdatingControls ) {
                updateHistogramLevels();
                emit display->update();
              }
            }
          }
        }
      });

  connect(spins[SPIN_HIGHLIGHTS], static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {

        if ( lidarDisplayView_ && !updatingControls_ ) {

          QLidarDisplay * display =
              lidarDisplayView_->currentLidarDisplay();

          if ( display ) {

            c_pixinsight_mtf & mtf =
                display->mtf();

            if ( v != mtf.highlights() ) {

              const bool wasInUpdatingControls =
              updatingControls();

              setUpdatingControls(true);

              mtf.set_highlights(v);
              mtfSlider_->setHighlights(mtf.highlights());

              setUpdatingControls(wasInUpdatingControls);
              if ( !wasInUpdatingControls ) {
                updateHistogramLevels();
                emit display->update();
              }
            }
          }
        }
      });

  connect(spins[SPIN_MIDTONES], static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
      [this](double v) {
        if ( lidarDisplayView_ && !updatingControls_ ) {

          QLidarDisplay * display =
              lidarDisplayView_->currentLidarDisplay();

          if ( display ) {
            c_pixinsight_mtf & mtf =
                display->mtf();

            if ( v != mtf.midtones() ) {

              const bool wasInUpdatingControls =
              updatingControls();

              setUpdatingControls(true);

              mtf.set_midtones(v);
              mtfSlider_->setMidtones(mtf.midtones());

              setUpdatingControls(wasInUpdatingControls);
              if ( !wasInUpdatingControls ) {
                updateHistogramLevels();
                emit display->update();
              }
            }
          }
        }
      });

  updateControls();
}

void QLidarDisplaySettingsWidget::setLidarDisplayView(QLidarDisplayView * displayView)
{

  if( lidarDisplayView_ ) {
    QLidarDisplay *currentDisplay = lidarDisplayView_->currentLidarDisplay();
    if( currentDisplay ) {
      disconnect(currentDisplay, &QLidarDisplay::update,
          this, &ThisClass::updateHistogramLevels);
    }
  }

  lidarDisplayView_ = displayView;
  updateControls();

  if( lidarDisplayView_ ) {
    QLidarDisplay *currentDisplay = lidarDisplayView_->currentLidarDisplay();
    if( currentDisplay ) {
      connect(currentDisplay, &QLidarDisplay::update,
          this, &ThisClass::updateHistogramLevels);
    }
  }
}

QLidarDisplayView * QLidarDisplaySettingsWidget::lidarDisplayView() const
{
  return lidarDisplayView_;
}

bool QLidarDisplaySettingsWidget::isAutoMtfActionEnabled() const
{
  return autoMtf_ctl && autoMtf_ctl->isChecked();
}

bool QLidarDisplaySettingsWidget::updatingControls() const
{
  return updatingControls_;
}

void QLidarDisplaySettingsWidget::setUpdatingControls(bool v)
{
  updatingControls_ = v;
}

void QLidarDisplaySettingsWidget::onChartTypeSelectorClicked()
{
  static QMenu chartTypesMenu;
  static QAction *setLineChartAction = Q_NULLPTR;
  static QAction *setBarChartAction = Q_NULLPTR;

  if( chartTypesMenu.isEmpty() ) {
    chartTypesMenu.addAction(setLineChartAction = new QAction(line_chart_icon, "Lines"));
    chartTypesMenu.addAction(setBarChartAction = new QAction(bar_chart_icon, "Bars"));
  }

  QAction *selectedAction =
      chartTypesMenu.exec(chartTypeSelectorButton_->mapToGlobal(QPoint(
          2 * chartTypeSelectorButton_->width() / 3,
          chartTypeSelectorButton_->height() / 2)));

  if( selectedAction ) {

    if( selectedAction == setLineChartAction ) {
      levelsView_->setChartType(QHistogramView::ChartType_Lines);
    }
    else if( selectedAction == setBarChartAction ) {
      levelsView_->setChartType(QHistogramView::ChartType_Bars);
    }

    chartTypeSelectorButton_->setIcon(selectChartTypeIcon(
        levelsView_->chartType()));
  }
}

void QLidarDisplaySettingsWidget::onResetMtfClicked()
{
  QLidarDisplay * display =
      lidarDisplayView_ ?
          lidarDisplayView_->currentLidarDisplay() :
          nullptr;

  if ( display ) {

    QWaitCursor wait(this);
    double min, max;

    const bool wasInUpdatingControls =
        updatingControls();

    setUpdatingControls(true);

    display->compute_input_data_range(&min, &max);

    c_pixinsight_mtf & mtf =
        display->mtf();
    mtf.set_input_range(min, max);
    mtf.set_shadows(0);
    mtf.set_highlights(1);
    mtf.set_midtones(0.5);
    display->save_paramters();

    updateControls();

    setUpdatingControls(wasInUpdatingControls);
    if ( !wasInUpdatingControls ) {
      updateHistogramLevels();
      emit display->update();
    }
  }
}

void QLidarDisplaySettingsWidget::onAutoMtfCtrlClicked()
{
  QLidarDisplay * display =
      lidarDisplayView_ ?
          lidarDisplayView_->currentLidarDisplay() :
          nullptr;

  if ( display ) {

    if ( autoMtf_ctl->isChecked() ) {

      switch ( selectedAutoMtfAction_ ) {
      case AutoMtfAction_AutoMtf :
        findAutoMidtonesBalance();
        break;

      case AutoMtfAction_AutoClip :
        default :
        findAutoHistogramClips();
        break;
      }
    }

    updateControls();

    if ( !updatingControls() ) {
      updateHistogramLevels();
      emit display->update();
    }
  }
}

void QLidarDisplaySettingsWidget::onColormapCtlClicked()
{
  QLidarDisplay *display =
      lidarDisplayView_ ?
          lidarDisplayView_->currentLidarDisplay() :
          nullptr;

  if( display ) {

    QMenu menu;

    const COLORMAP current_colormap =
        display->colormap();

    for( const c_enum_member *colormap = members_of<COLORMAP>(); colormap->name; ++colormap ) {

      QAction * action = new QAction(colormap->name);
      action->setData((int) (colormap->value));

      if( colormap->value == current_colormap ) {
        action->setCheckable(true);
        action->setChecked(true);
      }

      menu.addAction(action);
    }

    QAction *action =
        menu.exec(colormap_ctl->mapToGlobal(
            QPoint(0, 0)));

    if ( action ) {
      display->set_colormap((COLORMAP )action->data().toInt());
      display->save_paramters();
      updateHistogramLevels();
      updateColormapPixmap();
      emit display->update();
    }
  }
}

void QLidarDisplaySettingsWidget::onDisplayTypeCurrentItemChanged()
{
  if( lidarDisplayView_ && !updatingControls_ ) {

    QLidarDisplay * currentDisplay =
        lidarDisplayView_->currentLidarDisplay();

    if( currentDisplay ) {
      disconnect(currentDisplay, &QLidarDisplay::update,
          this, &ThisClass::updateHistogramLevels);
    }

    lidarDisplayView_->setCurrentLidarDisplay(currentDisplay =
        getLidarDisplay(displayType_ctl->currentItem()));

    updateControls();

    if( currentDisplay ) {
      emit lidarDisplayView_->currentLidarDisplay()->update();
      connect(currentDisplay, &QLidarDisplay::update,
          this, &ThisClass::updateHistogramLevels);
    }
  }
}

void QLidarDisplaySettingsWidget::onInputDataRangeChanged()
{
  if( lidarDisplayView_ && !updatingControls_ ) {

    QLidarDisplay *display =
        lidarDisplayView_->currentLidarDisplay();

    if( display ) {

      double range[2];

      if( fromString(inputDataRange_ctl->text(), range, 2) == 2 ) {

        c_pixinsight_mtf &mtf =
            display->mtf();

        const bool wasInUpdatingControls =
            updatingControls();

        setUpdatingControls(true);

        mtf.set_input_range(range[0], range[1]);
        display->save_paramters();

        setUpdatingControls(wasInUpdatingControls);

        if( !wasInUpdatingControls ) {
          updateHistogramLevels();
          emit display->update();
        }
      }
    }
  }
}



void QLidarDisplaySettingsWidget::findAutoHistogramClips()
{
  QLidarDisplay *display =
      lidarDisplayView_ ?
          lidarDisplayView_->currentLidarDisplay() :
          nullptr;

  if( display ) {

    QWaitCursor wait(this);

    c_pixinsight_mtf &mtf =
        display->mtf();

    double data_min = -1, data_max = -1;
    double range_min = -1, range_max = -1;

    display->compute_input_data_range(&data_min, &data_max);
    mtf.get_input_range(&range_min, &range_max);

    if ( data_min >= data_max ) {
      data_min = 0;
      data_max = 1;
    }

    if ( range_min >= range_max ) {
      range_min = 0;
      range_max = 1;
    }

    mtf.set_shadows((data_min - range_min) / (range_max - range_min));
    mtf.set_highlights((data_max - range_min) / (range_max - range_min));
    mtf.set_midtones(0.5);

    if( !updatingControls() ) {
      emit display->update();
    }
  }
}

void QLidarDisplaySettingsWidget::findAutoMidtonesBalance()
{
  QLidarDisplay *display =
      lidarDisplayView_ ?
          lidarDisplayView_->currentLidarDisplay() :
          nullptr;

  if( display ) {

    QWaitCursor wait(this);

    c_pixinsight_mtf &mtf =
        display->mtf();

    cv::Mat1f H;
    double hmin = -1, hmax = -1;

    display->create_input_histogramm(H, &hmin, &hmax);
    if( H.empty() ) {
      CF_ERROR("display->create_input_histogramm() fails");
    }
    else {
      mtf.find_midtones_balance(H);

      updateControls();

      if( !updatingControls() ) {
        emit display->update();
      }
    }
  }

}

void QLidarDisplaySettingsWidget::updateHistogramLevels()
{
  cv::Mat1f H;
  double hmin = -1, hmax = -1;

  QLidarDisplay *display =
      lidarDisplayView_ ?
          lidarDisplayView_->currentLidarDisplay() :
          nullptr;

  if( display ) {
    display->create_output_histogramm(H, &hmin, &hmax);
  }

  levelsView_->setHistogram(H, hmin, hmax);
}

void QLidarDisplaySettingsWidget::updateColormapPixmap()
{
  QLidarDisplay *display =
      lidarDisplayView_ ?
          lidarDisplayView_->currentLidarDisplay() :
          nullptr;

  if ( display ) {

    cv::Mat1b gray_image(16, 256);
    QImage qimage;

    for( int y = 0; y < gray_image.rows; ++y ) {
      for( int x = 0; x < gray_image.cols; ++x ) {
        gray_image[y][x] = x;
      }
    }

    const COLORMAP cmap =
        display->colormap();

    if ( cmap == COLORMAP_NONE ) {
      cv2qt(gray_image, &qimage);
    }
    else {
      cv::Mat3b color_image;
      apply_colormap(gray_image, color_image, cmap);
      cv2qt(color_image, &qimage);
    }

    colormap_pixmap_ = QPixmap::fromImage(qimage);
  }

  updateColormapStrip();

}

void QLidarDisplaySettingsWidget::updateColormapStrip()
{
  if ( colormap_strip_ctl ) {

    colormap_strip_ctl->setPixmap(colormap_pixmap_.scaled(
        mtfSlider_->width(), colormap_pixmap_.height()));

    colormap_strip_ctl->setMinimumSize(64, 16);
  }
}

void QLidarDisplaySettingsWidget::resizeEvent(QResizeEvent *event)
{
  Base::resizeEvent(event);
  updateColormapStrip();
}

void QLidarDisplaySettingsWidget::updateControls()
{
  const QLidarDisplay * display =
      lidarDisplayView_ ?
          lidarDisplayView_->currentLidarDisplay() :
          nullptr;

  if( !display ) {
    setEnabled(false);
  }
  else {

    double minval, maxval;

    const bool wasInUpdatingControls =
        updatingControls();

    setUpdatingControls(true);

    displayType_ctl->setCurrentItem(display->type());

    const c_pixinsight_mtf & mtf =
        display->mtf();

    mtf.get_input_range(&minval, &maxval);
    inputDataRange_ctl->setText(QString("%1;%2").arg(minval).arg(maxval));

    mtfSlider_->setup(mtf.shadows(), mtf.highlights(), mtf.midtones());

    spins[SPIN_SHADOWS]->setValue(mtf.shadows());
    spins[SPIN_HIGHLIGHTS]->setValue(mtf.highlights());
    spins[SPIN_MIDTONES]->setValue(mtf.midtones());
    logScaleSelectionAction_->setChecked(levelsView_->logScale());

    const bool autoMtfChecked =
        autoMtf_ctl->isChecked();

    spins[SPIN_SHADOWS]->setEnabled(!autoMtfChecked);
    spins[SPIN_HIGHLIGHTS]->setEnabled(!autoMtfChecked);
    spins[SPIN_MIDTONES]->setEnabled(!autoMtfChecked);
    mtfSlider_->setEnabled(!autoMtfChecked);

    switch (selectedAutoMtfAction_) {
    case AutoMtfAction_AutoMtf:
      autoMtf_ctl->setIcon(auto_mtf_icon);
      break;
    case AutoMtfAction_AutoClip:
      default:
      autoMtf_ctl->setIcon(auto_clip_icon);
      break;
    }

    updateHistogramLevels();
    setUpdatingControls(wasInUpdatingControls);
    updateColormapPixmap();
    setEnabled(true);
  }
}

QLidarDisplaySettings::QLidarDisplaySettings(QWidget * parent) :
    Base(parent)
{
  intit_mtfcontrols_resources();

  setWindowIcon(getIcon(ICON_histogram));
  setWindowTitle("Select visual data and adjust display levels ...");

  vbox_ = new QVBoxLayout(this);
  widget = new QLidarDisplaySettingsWidget(this);
  vbox_->addWidget(widget);
}

void QLidarDisplaySettings::setLidarDisplayView(QLidarDisplayView * displayView)
{
  widget->setLidarDisplayView(displayView);
}

QLidarDisplayView * QLidarDisplaySettings::lidarDisplayView() const
{
  return widget->lidarDisplayView();
}

void QLidarDisplaySettings::showEvent(QShowEvent *event)
{
  if( !lastWidnowSize_.isEmpty() ) {
    Base::move(lastWidnowPos_);
    Base::resize(lastWidnowSize_);
  }

  Base::showEvent(event);
  emit visibilityChanged(isVisible());
}

void QLidarDisplaySettings::hideEvent(QHideEvent *event)
{
  lastWidnowSize_ = this->size();
  lastWidnowPos_ = this->pos();

  Base::hideEvent(event);
  emit visibilityChanged(isVisible());
}
