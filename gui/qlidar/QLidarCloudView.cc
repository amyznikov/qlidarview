/*
 * QLidarCloudView.cc
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#include "QLidarCloudView.h"
#include <core/ssprintf.h>
#include <core/debug.h>


///////////////////////////////////////////////////////////////////////////////

bool fromString(const QString & s, QGLVector * v)
{
  return sscanf(s.toUtf8().data(), "%lf ; %lf ; %lf", &v->x, &v->y, &v->z) == 3;
}

QString toString(const QGLVector & v)
{
  return QString("%1;%2;%3").arg(v.x).arg(v.y).arg(v.z);
}

bool load_parameter(const QSettings & settings, const QString & prefix, const char * name,  QGLVector * v)
{
  return fromString(settings.value(QString("%1/%2").arg(prefix).arg(name), "").toString(), v);
}

void save_parameter(const QString & prefix, const char * name, const QGLVector & value )
{
  QSettings settings;
  settings.setValue(QString("%1/%2").arg(prefix).arg(name), toString(value));
}


///////////////////////////////////////////////////////////////////////////////


QLidarCloudView::QLidarCloudView(QWidget* parent)
  : Base(parent)
{
  setSceneRadius(100);

  QSettings settings;

  const LIDAR_DISPLAY_TYPE displayType =
      fromString(settings.value("CloudViewLidarDisplay").toString().toStdString(),
          LIDAR_DISPLAY_DEPTH);

  setCurrentLidarDisplay(getLidarDisplay(displayType));
}

void QLidarCloudView::setSceneRadius(qreal radius)
{
  Base::setSceneRadius(radius);
  update();
}

void QLidarCloudView::setSceneOrigin(const QGLVector & v)
{
  sceneOrigin_ = v;
  update();
}

QGLVector QLidarCloudView::sceneOrigin() const
{
  return sceneOrigin_;
}

void QLidarCloudView::setPointSize(double v)
{
  pointSize_ = v;
  update();
}

double QLidarCloudView::pointSize() const
{
  return pointSize_;
}

void QLidarCloudView::setCurrentLidarDisplay(QLidarDisplay * display)
{
  if( currentLidarDisplay_ ) {
    disconnect(currentLidarDisplay_, &QLidarDisplay::update,
        this, &ThisClass::updatePointCloud);
  }

  if ( (currentLidarDisplay_ = display) ) {

    connect(currentLidarDisplay_, &QLidarDisplay::update,
        this, &ThisClass::updatePointCloud);

    QSettings settings;
    settings.setValue("CloudViewLidarDisplay",
        toString(currentLidarDisplay_->type()));
  }

  update();
}

void QLidarCloudView::updatePointCloud()
{
  positions_.clear();
  colors_.clear();

  if ( currentLidarDisplay_ ) {
    currentLidarDisplay_->create_point_cloud(
        &positions_,
        &colors_);
  }

  update();
}



void QLidarCloudView::init()
{
  setSceneCenter(QGLVector(0.0, 0.0, 0.0));
  setBackgroundColor(QColor(0, 0, 0));

  glDisable(GL_LIGHTING);
  glDisable(GL_COLOR_MATERIAL);
  glEnable(GL_PROGRAM_POINT_SIZE);

  Base::init();
}


void QLidarCloudView::draw()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(0.0, 0.0, 0.0, 1.0);

  glPointSize(pointSize_);

  glBegin(GL_POINTS);

  for( int i = 0, n = positions_.size(); i < n; ++i ) {

    const cv::Vec3f & pos = positions_[i];
    const cv::Vec3b & color = colors_[i];

    float x = pos(0);
    float y = pos(1);
    float z = pos(2);

    glColor3ub(color(2), color(1), color(0));
    glVertex3f(x - sceneOrigin_.x, y - sceneOrigin_.y, z - sceneOrigin_.z);
  }

  glEnd(/*GL_POINTS*/);
}


