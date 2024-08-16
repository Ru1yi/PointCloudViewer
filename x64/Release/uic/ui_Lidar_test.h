/********************************************************************************
** Form generated from reading UI file 'Lidar_test.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LIDAR_TEST_H
#define UI_LIDAR_TEST_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Lidar_testClass
{
public:
    QWidget *centralWidget;
    QMenuBar *menuBar;
    QMenu *menu;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *Lidar_testClass)
    {
        if (Lidar_testClass->objectName().isEmpty())
            Lidar_testClass->setObjectName(QString::fromUtf8("Lidar_testClass"));
        Lidar_testClass->resize(600, 400);
        centralWidget = new QWidget(Lidar_testClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        Lidar_testClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(Lidar_testClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 17));
        menu = new QMenu(menuBar);
        menu->setObjectName(QString::fromUtf8("menu"));
        Lidar_testClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(Lidar_testClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        Lidar_testClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(Lidar_testClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        Lidar_testClass->setStatusBar(statusBar);

        menuBar->addAction(menu->menuAction());

        retranslateUi(Lidar_testClass);

        QMetaObject::connectSlotsByName(Lidar_testClass);
    } // setupUi

    void retranslateUi(QMainWindow *Lidar_testClass)
    {
        Lidar_testClass->setWindowTitle(QCoreApplication::translate("Lidar_testClass", "Lidar_test", nullptr));
        menu->setTitle(QCoreApplication::translate("Lidar_testClass", "\346\277\200\345\205\211\351\233\267\350\276\276\347\202\271\344\272\221\345\217\257\350\247\206\345\214\226 V1.0", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Lidar_testClass: public Ui_Lidar_testClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LIDAR_TEST_H
