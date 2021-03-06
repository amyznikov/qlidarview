/*
 * main.cc
 *
 *  Created on September 5, 2019
 *      Author: amyznikov
 */
//#include <opencv2/core/ocl.hpp>
#include "MainWindow.h"
#include <core/debug.h>


#define MY_COMPANY  "amyznikov"
#define MY_APP      "qlidarview"

using namespace qlidarview;

int main(int argc, char *argv[])
{
  // My OpenCV build has problems with OCL.
  // Comment out this line if you want to allow OCL in OpenCV
  // cv::ocl::setUseOpenCL(false);


  QApplication app(argc, argv);
  app.setOrganizationName(MY_COMPANY);
  app.setApplicationName(MY_APP);

  Q_INIT_RESOURCE(app_resources);
  Q_INIT_RESOURCE(gui_resources);

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);
  cf_enable_error_log(true);


  MainWindow mainWindow;
  mainWindow.show();
  mainWindow.activateWindow();
  mainWindow.raise();

  return app.exec();
}
