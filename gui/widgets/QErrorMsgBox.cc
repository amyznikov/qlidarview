/*
 * QErrorMsgBox.cc
 *
 *  Created on: Mar 24, 2022
 *      Author: amyznikov
 */

#include "QErrorMsgBox.h"
#include <core/debug.h>

QErrorMsgBox::QErrorMsgBox(QWidget * parent, const QString & title, const QString & msg, bool enable_cf_errlog)
  : Base(parent)
{
  QVBoxLayout * vbox = Q_NULLPTR;
  QHBoxLayout * hbox = Q_NULLPTR;

  QIcon icon =
      qApp->style()->standardIcon(QStyle::SP_MessageBoxCritical);

  setWindowTitle(title);
  setWindowIcon(icon);
  setMinimumWidth(320);

  vbox = new QVBoxLayout(this);

  hbox = new QHBoxLayout();
  erricon_ctl = new QLabel();
  erricon_ctl->setPixmap(icon.pixmap(QSize(32, 32)));
  hbox->addWidget(erricon_ctl, 0, Qt::AlignLeft);

  errmsg_ctl = new QLabel();
  errmsg_ctl->setTextFormat(Qt::RichText);
  errmsg_ctl->setWordWrap(true);
  errmsg_ctl->setAlignment(Qt::AlignTop);
  errmsg_ctl->setTextInteractionFlags(
      Qt::TextSelectableByKeyboard|
      Qt::TextSelectableByMouse|
      Qt::LinksAccessibleByMouse|
      Qt::LinksAccessibleByKeyboard);

  errmsg_ctl->setText(QString("<strong>%1</strong>").arg(msg));
  hbox->addWidget(errmsg_ctl, 100);
  vbox->addLayout(hbox, 1);

  if ( !enable_cf_errlog ) {
    // button box
    hbox = new QHBoxLayout();
  }
  else {

    QString lasterrmsg = cf_get_last_error_msg().c_str();
    if ( !lasterrmsg.isEmpty() ) {

      hbox = new QHBoxLayout();
      lasterr_ctl = new QLabel();
      lasterr_ctl->setTextFormat(Qt::RichText);
      lasterr_ctl->setWordWrap(true);
      lasterr_ctl->setTextInteractionFlags(
          Qt::TextSelectableByKeyboard|
          Qt::TextSelectableByMouse|
          Qt::LinksAccessibleByMouse|
          Qt::LinksAccessibleByKeyboard);
      lasterr_ctl->setText(lasterrmsg);
      hbox->addWidget(lasterr_ctl, 100);
      vbox->addLayout(hbox, 10);
    }

    hbox = new QHBoxLayout();
  }

  closeButton = new QPushButton("Close");
  closeButton->setDefault(true);

  hbox->addWidget(closeButton, 1, Qt::AlignRight);

  connect(closeButton, &QPushButton::clicked,
      this, &ThisClass::accept);

  std::vector<cf_error_log_entry> errlog;
  cf_get_error_log(&errlog);

  if( !errlog.empty() ) {
    detailsButton = new QPushButton("Show details...");
    hbox->insertWidget(0, detailsButton, 0, Qt::AlignRight);
    connect(detailsButton, &QPushButton::clicked,
        [this]() {
          if ( errlog_ctl ) {
            errlog_ctl->setVisible(!errlog_ctl->isVisible());
            detailsButton->setText(errlog_ctl->isVisible() ? "Hide details" : "Show details...");
            adjustSize();
          }
        });
  }
  vbox->addLayout(hbox, 1);

  if( !errlog.empty() ) {

    QString details;

    for( int i = errlog.size() - 1; i >= 0; --i ) {
      const cf_error_log_entry &e = errlog[i];
      details.append(QString("%1\n%2() :\n     %3\n\n").
          arg(e.file.c_str()).
          arg(e.func.c_str()).
          arg(e.msg.c_str()));
    }

    errlog_ctl = new QTextBrowser();
    errlog_ctl->setText(details);
    vbox->addWidget(errlog_ctl, 1000);

    errlog_ctl->setVisible(false);
  }
}

void QErrorMsgBox::show(QWidget * parent, const QString & title, const QString & msg, bool enable_cf_errlog)
{
  QErrorMsgBox dlgbox(parent, title, msg, enable_cf_errlog);
  dlgbox.exec();
}

//
//void QErrorMsgBox::show(QWidget * parent, const QString & title, const QString & msg, bool enable_cf_errlog)
//{
//  QMessageBox dlgbox(
//      QMessageBox::Icon::Critical,
//      title,
//      "",
//      QMessageBox::StandardButton::Ok,
//      parent);
//
//  //dlgbox.setTextFormat(Qt::RichText);
//
//  dlgbox.setTextInteractionFlags(
//      Qt::TextSelectableByKeyboard |
//          Qt::TextSelectableByMouse |
//          Qt::LinksAccessibleByMouse |
//          Qt::LinksAccessibleByKeyboard);
//
//
//
//
//
//  QString informativeText =
//      cf_get_last_error_msg().c_str();
//
//  if ( informativeText.isEmpty() ) {
//    dlgbox.setText(msg);
//  }
//  else {
//    dlgbox.setText(msg + " :\n");
//    dlgbox.setInformativeText(informativeText);
//  }
//
//  if ( enable_cf_errlog ) {
//
//    std::vector<cf_error_log_entry> errlog;
//    cf_get_error_log(&errlog);
//
//    if ( !errlog.empty() ) {
//
//      QString details;
//
//      for( int i = errlog.size() - 1; i >= 0; --i ) {
//        const cf_error_log_entry &e = errlog[i];
//        details.append(QString("%1\n%2() :\n     %3\n\n").
//            arg(e.file.c_str()).
//            arg(e.func.c_str()).
//            arg(e.msg.c_str()));
//      }
//
//      dlgbox.setDetailedText(details);
//    }
//  }
//
//  dlgbox.exec();
//}
