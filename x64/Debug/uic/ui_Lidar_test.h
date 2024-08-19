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
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Lidar_testClass
{
public:
    QAction *actionopen;
    QWidget *centralWidget;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *lineEdit_color;
    QPushButton *pushButton_ChangeColor;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QComboBox *comboBox_View;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_GetCurrentView;
    QTextEdit *textEdit_GetCurrentView;
    QWidget *horizontalLayoutWidget_3;
    QHBoxLayout *horizontalLayout_3;
    QCheckBox *checkBox_Loop;
    QCheckBox *checkBox_Grid;
    QPushButton *pushButton_ChangeColor_2;
    QMenuBar *menuBar;
    QMenu *menu;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *Lidar_testClass)
    {
        if (Lidar_testClass->objectName().isEmpty())
            Lidar_testClass->setObjectName(QString::fromUtf8("Lidar_testClass"));
        Lidar_testClass->resize(2260, 1080);
        actionopen = new QAction(Lidar_testClass);
        actionopen->setObjectName(QString::fromUtf8("actionopen"));
        centralWidget = new QWidget(Lidar_testClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        horizontalLayoutWidget = new QWidget(centralWidget);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(20, 220, 631, 101));
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

        pushButton_ChangeColor = new QPushButton(horizontalLayoutWidget);
        pushButton_ChangeColor->setObjectName(QString::fromUtf8("pushButton_ChangeColor"));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Times New Roman"));
        font1.setPointSize(12);
        font1.setBold(true);
        pushButton_ChangeColor->setFont(font1);
        pushButton_ChangeColor->setFlat(false);

        horizontalLayout->addWidget(pushButton_ChangeColor);

        horizontalLayoutWidget_2 = new QWidget(centralWidget);
        horizontalLayoutWidget_2->setObjectName(QString::fromUtf8("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(20, 80, 911, 141));
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

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        pushButton_GetCurrentView = new QPushButton(horizontalLayoutWidget_2);
        pushButton_GetCurrentView->setObjectName(QString::fromUtf8("pushButton_GetCurrentView"));
        pushButton_GetCurrentView->setFont(font1);

        verticalLayout->addWidget(pushButton_GetCurrentView);

        textEdit_GetCurrentView = new QTextEdit(horizontalLayoutWidget_2);
        textEdit_GetCurrentView->setObjectName(QString::fromUtf8("textEdit_GetCurrentView"));

        verticalLayout->addWidget(textEdit_GetCurrentView);


        horizontalLayout_2->addLayout(verticalLayout);

        horizontalLayoutWidget_3 = new QWidget(centralWidget);
        horizontalLayoutWidget_3->setObjectName(QString::fromUtf8("horizontalLayoutWidget_3"));
        horizontalLayoutWidget_3->setGeometry(QRect(20, 0, 271, 80));
        horizontalLayout_3 = new QHBoxLayout(horizontalLayoutWidget_3);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        checkBox_Loop = new QCheckBox(horizontalLayoutWidget_3);
        checkBox_Loop->setObjectName(QString::fromUtf8("checkBox_Loop"));
        checkBox_Loop->setFont(font);

        horizontalLayout_3->addWidget(checkBox_Loop);

        checkBox_Grid = new QCheckBox(horizontalLayoutWidget_3);
        checkBox_Grid->setObjectName(QString::fromUtf8("checkBox_Grid"));
        checkBox_Grid->setFont(font);

        horizontalLayout_3->addWidget(checkBox_Grid);

        pushButton_ChangeColor_2 = new QPushButton(centralWidget);
        pushButton_ChangeColor_2->setObjectName(QString::fromUtf8("pushButton_ChangeColor_2"));
        pushButton_ChangeColor_2->setGeometry(QRect(20, 340, 211, 41));
        pushButton_ChangeColor_2->setFont(font1);
        pushButton_ChangeColor_2->setFlat(false);
        Lidar_testClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(Lidar_testClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 2260, 17));
        menu = new QMenu(menuBar);
        menu->setObjectName(QString::fromUtf8("menu"));
        QFont font4;
        font4.setPointSize(18);
        font4.setBold(true);
        menu->setFont(font4);
        Lidar_testClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(Lidar_testClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        Lidar_testClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(Lidar_testClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        Lidar_testClass->setStatusBar(statusBar);

        menuBar->addAction(menu->menuAction());
        menu->addAction(actionopen);

        retranslateUi(Lidar_testClass);
        QObject::connect(pushButton_ChangeColor, SIGNAL(clicked()), Lidar_testClass, SLOT(on_PushButton_ChangeColor_Clicked()));
        QObject::connect(comboBox_View, SIGNAL(currentIndexChanged(int)), Lidar_testClass, SLOT(on_ComboBox_View_Changed()));
        QObject::connect(checkBox_Grid, SIGNAL(stateChanged(int)), Lidar_testClass, SLOT(on_CheckBox_Grid_stateChanged()));
        QObject::connect(pushButton_GetCurrentView, SIGNAL(clicked()), Lidar_testClass, SLOT(on_PushButton_GetCurrentView_Clicked()));
        QObject::connect(pushButton_ChangeColor_2, SIGNAL(clicked()), Lidar_testClass, SLOT(on_PushButton_AdjustPC_Clicked()));

        pushButton_ChangeColor->setDefault(false);
        pushButton_ChangeColor_2->setDefault(false);


        QMetaObject::connectSlotsByName(Lidar_testClass);
    } // setupUi

    void retranslateUi(QMainWindow *Lidar_testClass)
    {
        Lidar_testClass->setWindowTitle(QCoreApplication::translate("Lidar_testClass", "Lidar_test", nullptr));
        actionopen->setText(QCoreApplication::translate("Lidar_testClass", "open", nullptr));
        label->setText(QCoreApplication::translate("Lidar_testClass", "Current Color Field\357\274\232", nullptr));
        pushButton_ChangeColor->setText(QCoreApplication::translate("Lidar_testClass", "Change", nullptr));
        label_2->setText(QCoreApplication::translate("Lidar_testClass", "Current View\357\274\232", nullptr));
        comboBox_View->setItemText(0, QCoreApplication::translate("Lidar_testClass", "Top View", nullptr));
        comboBox_View->setItemText(1, QCoreApplication::translate("Lidar_testClass", "Special View", nullptr));

        pushButton_GetCurrentView->setText(QCoreApplication::translate("Lidar_testClass", "Get Current View", nullptr));
        checkBox_Loop->setText(QCoreApplication::translate("Lidar_testClass", "loop", nullptr));
        checkBox_Grid->setText(QCoreApplication::translate("Lidar_testClass", "grid", nullptr));
        pushButton_ChangeColor_2->setText(QCoreApplication::translate("Lidar_testClass", "Adjust PointCloud", nullptr));
        menu->setTitle(QCoreApplication::translate("Lidar_testClass", "INVENT PointCloud Viewer V1.0", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Lidar_testClass: public Ui_Lidar_testClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LIDAR_TEST_H
