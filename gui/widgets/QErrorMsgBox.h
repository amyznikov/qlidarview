/*
 * QErrorMsgBox.h
 *
 *  Created on: Mar 24, 2022
 *      Author: amyznikov
 */

#ifndef __QErrorMsgBox_h__
#define __QErrorMsgBox_h__

#include <QtWidgets/QtWidgets>

class QErrorMsgBox:
    public QDialog
{
  Q_OBJECT;
public:
  typedef QErrorMsgBox ThisClass;
  typedef QDialog Base;

  static void show(QWidget * parent,
      const QString & title,
      const QString & msg,
      bool enable_cf_errlog = true);

protected:
  QErrorMsgBox(QWidget * parent,
      const QString & title,
      const QString & msg,
      bool enable_cf_errlog = true);

protected:
  QLabel * erricon_ctl = Q_NULLPTR;
  QLabel * errmsg_ctl = Q_NULLPTR;
  QLabel * lasterr_ctl = Q_NULLPTR;
  QTextBrowser * errlog_ctl = Q_NULLPTR;
  QPushButton * closeButton = Q_NULLPTR;
  QPushButton * detailsButton = Q_NULLPTR;
};

//
//class QErrorMsgBox
//{
//public:
//  static void show(QWidget * parent,
//      const QString & title,
//      const QString & msg,
//      bool enable_cf_errlog = true);
//};

#endif /* __QErrorMsgBox_h__ */
