/********************************************************************************
** Form generated from reading UI file 'QtWidgetsClass123.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QTWIDGETSCLASS123_H
#define UI_QTWIDGETSCLASS123_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_QtWidgetsClass123Class
{
public:
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QWidget *centralWidget;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *QtWidgetsClass123Class)
    {
        if (QtWidgetsClass123Class->objectName().isEmpty())
            QtWidgetsClass123Class->setObjectName(QString::fromUtf8("QtWidgetsClass123Class"));
        QtWidgetsClass123Class->resize(600, 400);
        menuBar = new QMenuBar(QtWidgetsClass123Class);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        QtWidgetsClass123Class->setMenuBar(menuBar);
        mainToolBar = new QToolBar(QtWidgetsClass123Class);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        QtWidgetsClass123Class->addToolBar(mainToolBar);
        centralWidget = new QWidget(QtWidgetsClass123Class);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        QtWidgetsClass123Class->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(QtWidgetsClass123Class);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        QtWidgetsClass123Class->setStatusBar(statusBar);

        retranslateUi(QtWidgetsClass123Class);

        QMetaObject::connectSlotsByName(QtWidgetsClass123Class);
    } // setupUi

    void retranslateUi(QMainWindow *QtWidgetsClass123Class)
    {
        QtWidgetsClass123Class->setWindowTitle(QCoreApplication::translate("QtWidgetsClass123Class", "QtWidgetsClass123", nullptr));
    } // retranslateUi

};

namespace Ui {
    class QtWidgetsClass123Class: public Ui_QtWidgetsClass123Class {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QTWIDGETSCLASS123_H
