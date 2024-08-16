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
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Lidar_testClass
{
public:
    QWidget *centralWidget;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *lineEdit_color;
    QPushButton *pushButton;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QComboBox *comboBox_View;
    QCheckBox *checkBox_loop;
    QMenuBar *menuBar;
    QMenu *menu;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *Lidar_testClass)
    {
        if (Lidar_testClass->objectName().isEmpty())
            Lidar_testClass->setObjectName(QString::fromUtf8("Lidar_testClass"));
        Lidar_testClass->resize(1920, 1080);
        centralWidget = new QWidget(Lidar_testClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        horizontalLayoutWidget = new QWidget(centralWidget);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(0, 80, 631, 101));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setSizeConstraint(QLayout::SetNoConstraint);
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(horizontalLayoutWidget);
        label->setObjectName(QString::fromUtf8("label"));
        QFont font;
        font.setFamily(QString::fromUtf8("Times New Roman"));
        font.setPointSize(14);
        font.setBold(true);
        label->setFont(font);

        horizontalLayout->addWidget(label);

        lineEdit_color = new QLineEdit(horizontalLayoutWidget);
        lineEdit_color->setObjectName(QString::fromUtf8("lineEdit_color"));

        horizontalLayout->addWidget(lineEdit_color);

        pushButton = new QPushButton(horizontalLayoutWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Times New Roman"));
        font1.setPointSize(12);
        pushButton->setFont(font1);
        pushButton->setFlat(false);

        horizontalLayout->addWidget(pushButton);

        horizontalLayoutWidget_2 = new QWidget(centralWidget);
        horizontalLayoutWidget_2->setObjectName(QString::fromUtf8("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(0, 0, 471, 80));
        horizontalLayout_2 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setSizeConstraint(QLayout::SetDefaultConstraint);
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_2 = new QLabel(horizontalLayoutWidget_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        QFont font2;
        font2.setFamily(QString::fromUtf8("Times New Roman"));
        font2.setPointSize(14);
        font2.setBold(true);
        font2.setItalic(false);
        label_2->setFont(font2);

        horizontalLayout_2->addWidget(label_2);

        comboBox_View = new QComboBox(horizontalLayoutWidget_2);
        comboBox_View->addItem(QString());
        comboBox_View->addItem(QString());
        comboBox_View->setObjectName(QString::fromUtf8("comboBox_View"));
        QFont font3;
        font3.setFamily(QString::fromUtf8("Times New Roman"));
        font3.setPointSize(14);
        comboBox_View->setFont(font3);

        horizontalLayout_2->addWidget(comboBox_View);

        checkBox_loop = new QCheckBox(centralWidget);
        checkBox_loop->setObjectName(QString::fromUtf8("checkBox_loop"));
        checkBox_loop->setGeometry(QRect(490, 20, 91, 31));
        checkBox_loop->setFont(font);
        Lidar_testClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(Lidar_testClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1920, 17));
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
        QObject::connect(pushButton, SIGNAL(clicked()), Lidar_testClass, SLOT(on_PushButton_Clicked()));
        QObject::connect(comboBox_View, SIGNAL(currentIndexChanged(int)), Lidar_testClass, SLOT(on_ComboBox_View_Changed()));

        pushButton->setDefault(false);


        QMetaObject::connectSlotsByName(Lidar_testClass);
    } // setupUi

    void retranslateUi(QMainWindow *Lidar_testClass)
    {
        Lidar_testClass->setWindowTitle(QCoreApplication::translate("Lidar_testClass", "Lidar_test", nullptr));
        label->setText(QCoreApplication::translate("Lidar_testClass", "Current Color Field\357\274\232", nullptr));
        pushButton->setText(QCoreApplication::translate("Lidar_testClass", "Change", nullptr));
        label_2->setText(QCoreApplication::translate("Lidar_testClass", "Current View\357\274\232", nullptr));
        comboBox_View->setItemText(0, QCoreApplication::translate("Lidar_testClass", "Top View", nullptr));
        comboBox_View->setItemText(1, QCoreApplication::translate("Lidar_testClass", "Special View", nullptr));

        checkBox_loop->setText(QCoreApplication::translate("Lidar_testClass", "loop", nullptr));
        menu->setTitle(QCoreApplication::translate("Lidar_testClass", "Invent PointCloud Viewer V1.0", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Lidar_testClass: public Ui_Lidar_testClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LIDAR_TEST_H
