/*
 * QLidarCloudView.h
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCloudViewer_h__
#define __QCloudViewer_h__

#include <QtWidgets/QtWidgets>
#include <QtOpenGL/QGLWidget>
#include <QGLViewer/qglviewer.h>
#include "QLidarDisplayView.h"

using QGLVector = qglviewer::Vec;

class QLidarCloudView :
    public QGLViewer,
    public QLidarDisplayView
{
  Q_OBJECT;
public:
  typedef QLidarCloudView ThisClass;
  typedef QGLViewer Base;

  QLidarCloudView(QWidget* parent = Q_NULLPTR);

  void setCurrentLidarDisplay(QLidarDisplay * display) override;

  void setSceneRadius(qreal radius) override;

  void setPointSize(double v);
  double pointSize() const;

  void setSceneOrigin(const QGLVector & v);
  QGLVector sceneOrigin() const;


protected:
  void init() override;
  void draw() override;
  void keyPressEvent(QKeyEvent *) override;
  void keyReleaseEvent(QKeyEvent *e) override;

protected slots:
  void updatePointCloud();


protected:
  std::vector<cv::Vec3f> positions_;
  std::vector<cv::Vec3b> colors_;

  QGLVector sceneOrigin_;
  double pointSize_ = 3;
  double pointBrightness_ = 0;
};


bool fromString(const QString & s, QGLVector * v);
QString toString(const QGLVector & v);

bool load_parameter(const QSettings & settings, const QString & prefix, const char * name,  QGLVector * v);
void save_parameter(const QString & prefix, const char * name, const QGLVector & value );

#endif /* __QCloudViewer_h__ */
