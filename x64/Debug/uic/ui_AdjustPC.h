/********************************************************************************
** Form generated from reading UI file 'AdjustPC.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ADJUSTPC_H
#define UI_ADJUSTPC_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_AdjustPCClass
{
public:
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_6;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_6;
    QSpacerItem *verticalSpacer;
    QVBoxLayout *verticalLayout;
    QSpacerItem *verticalSpacer_2;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_5;
    QSlider *horizontalSlider_x;
    QLineEdit *lineEdit_x;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_4;
    QSlider *horizontalSlider_y;
    QLineEdit *lineEdit_y;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QSlider *horizontalSlider_z;
    QLineEdit *lineEdit_z;
    QHBoxLayout *horizontalLayout_7;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_7;
    QSpacerItem *verticalSpacer_3;
    QVBoxLayout *verticalLayout_2;
    QSpacerItem *verticalSpacer_4;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_8;
    QSlider *horizontalSlider_roll;
    QLineEdit *lineEdit_roll;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_9;
    QSlider *horizontalSlider_pitch;
    QLineEdit *lineEdit_pitch;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_10;
    QSlider *horizontalSlider_yaw;
    QLineEdit *lineEdit_yaw;

    void setupUi(QWidget *AdjustPCClass)
    {
        if (AdjustPCClass->objectName().isEmpty())
            AdjustPCClass->setObjectName(QString::fromUtf8("AdjustPCClass"));
        AdjustPCClass->resize(871, 649);
        verticalLayoutWidget_3 = new QWidget(AdjustPCClass);
        verticalLayoutWidget_3->setObjectName(QString::fromUtf8("verticalLayoutWidget_3"));
        verticalLayoutWidget_3->setGeometry(QRect(30, 180, 711, 271));
        verticalLayout_5 = new QVBoxLayout(verticalLayoutWidget_3);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        label_6 = new QLabel(verticalLayoutWidget_3);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        QFont font;
        font.setFamily(QString::fromUtf8("Times New Roman"));
        font.setPointSize(14);
        font.setBold(true);
        label_6->setFont(font);

        verticalLayout_3->addWidget(label_6);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);


        horizontalLayout_6->addLayout(verticalLayout_3);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalSpacer_2 = new QSpacerItem(20, 30, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(20);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_5 = new QLabel(verticalLayoutWidget_3);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Times New Roman"));
        font1.setPointSize(12);
        font1.setBold(true);
        label_5->setFont(font1);

        horizontalLayout_5->addWidget(label_5);

        horizontalSlider_x = new QSlider(verticalLayoutWidget_3);
        horizontalSlider_x->setObjectName(QString::fromUtf8("horizontalSlider_x"));
        horizontalSlider_x->setMaximum(9999);
        horizontalSlider_x->setValue(5000);
        horizontalSlider_x->setOrientation(Qt::Horizontal);

        horizontalLayout_5->addWidget(horizontalSlider_x);

        lineEdit_x = new QLineEdit(verticalLayoutWidget_3);
        lineEdit_x->setObjectName(QString::fromUtf8("lineEdit_x"));
        lineEdit_x->setAlignment(Qt::AlignCenter);
        lineEdit_x->setClearButtonEnabled(true);

        horizontalLayout_5->addWidget(lineEdit_x);

        horizontalLayout_5->setStretch(0, 1);
        horizontalLayout_5->setStretch(1, 18);
        horizontalLayout_5->setStretch(2, 6);

        verticalLayout->addLayout(horizontalLayout_5);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(20);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_4 = new QLabel(verticalLayoutWidget_3);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setFont(font1);

        horizontalLayout_4->addWidget(label_4);

        horizontalSlider_y = new QSlider(verticalLayoutWidget_3);
        horizontalSlider_y->setObjectName(QString::fromUtf8("horizontalSlider_y"));
        horizontalSlider_y->setMaximum(9999);
        horizontalSlider_y->setValue(5000);
        horizontalSlider_y->setOrientation(Qt::Horizontal);

        horizontalLayout_4->addWidget(horizontalSlider_y);

        lineEdit_y = new QLineEdit(verticalLayoutWidget_3);
        lineEdit_y->setObjectName(QString::fromUtf8("lineEdit_y"));
        lineEdit_y->setAlignment(Qt::AlignCenter);
        lineEdit_y->setClearButtonEnabled(true);

        horizontalLayout_4->addWidget(lineEdit_y);

        horizontalLayout_4->setStretch(0, 1);
        horizontalLayout_4->setStretch(1, 18);
        horizontalLayout_4->setStretch(2, 6);

        verticalLayout->addLayout(horizontalLayout_4);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(20);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(verticalLayoutWidget_3);
        label->setObjectName(QString::fromUtf8("label"));
        label->setFont(font1);

        horizontalLayout->addWidget(label);

        horizontalSlider_z = new QSlider(verticalLayoutWidget_3);
        horizontalSlider_z->setObjectName(QString::fromUtf8("horizontalSlider_z"));
        horizontalSlider_z->setMaximum(9999);
        horizontalSlider_z->setValue(5000);
        horizontalSlider_z->setOrientation(Qt::Horizontal);

        horizontalLayout->addWidget(horizontalSlider_z);

        lineEdit_z = new QLineEdit(verticalLayoutWidget_3);
        lineEdit_z->setObjectName(QString::fromUtf8("lineEdit_z"));
        lineEdit_z->setAlignment(Qt::AlignCenter);
        lineEdit_z->setClearButtonEnabled(true);

        horizontalLayout->addWidget(lineEdit_z);

        horizontalLayout->setStretch(0, 1);
        horizontalLayout->setStretch(1, 18);
        horizontalLayout->setStretch(2, 6);

        verticalLayout->addLayout(horizontalLayout);


        horizontalLayout_6->addLayout(verticalLayout);


        verticalLayout_5->addLayout(horizontalLayout_6);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(30);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        label_7 = new QLabel(verticalLayoutWidget_3);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setFont(font);

        verticalLayout_4->addWidget(label_7);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer_3);


        horizontalLayout_7->addLayout(verticalLayout_4);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalSpacer_4 = new QSpacerItem(20, 30, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_4);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(20);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        label_8 = new QLabel(verticalLayoutWidget_3);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setFont(font1);

        horizontalLayout_8->addWidget(label_8);

        horizontalSlider_roll = new QSlider(verticalLayoutWidget_3);
        horizontalSlider_roll->setObjectName(QString::fromUtf8("horizontalSlider_roll"));
        horizontalSlider_roll->setMaximum(9999);
        horizontalSlider_roll->setValue(5000);
        horizontalSlider_roll->setOrientation(Qt::Horizontal);

        horizontalLayout_8->addWidget(horizontalSlider_roll);

        lineEdit_roll = new QLineEdit(verticalLayoutWidget_3);
        lineEdit_roll->setObjectName(QString::fromUtf8("lineEdit_roll"));
        lineEdit_roll->setAlignment(Qt::AlignCenter);
        lineEdit_roll->setClearButtonEnabled(true);

        horizontalLayout_8->addWidget(lineEdit_roll);

        horizontalLayout_8->setStretch(0, 1);
        horizontalLayout_8->setStretch(1, 18);
        horizontalLayout_8->setStretch(2, 6);

        verticalLayout_2->addLayout(horizontalLayout_8);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(20);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_9 = new QLabel(verticalLayoutWidget_3);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setFont(font1);

        horizontalLayout_9->addWidget(label_9);

        horizontalSlider_pitch = new QSlider(verticalLayoutWidget_3);
        horizontalSlider_pitch->setObjectName(QString::fromUtf8("horizontalSlider_pitch"));
        horizontalSlider_pitch->setMaximum(9999);
        horizontalSlider_pitch->setValue(5000);
        horizontalSlider_pitch->setOrientation(Qt::Horizontal);

        horizontalLayout_9->addWidget(horizontalSlider_pitch);

        lineEdit_pitch = new QLineEdit(verticalLayoutWidget_3);
        lineEdit_pitch->setObjectName(QString::fromUtf8("lineEdit_pitch"));
        lineEdit_pitch->setAlignment(Qt::AlignCenter);
        lineEdit_pitch->setClearButtonEnabled(true);

        horizontalLayout_9->addWidget(lineEdit_pitch);

        horizontalLayout_9->setStretch(0, 1);
        horizontalLayout_9->setStretch(1, 18);
        horizontalLayout_9->setStretch(2, 6);

        verticalLayout_2->addLayout(horizontalLayout_9);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(20);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        label_10 = new QLabel(verticalLayoutWidget_3);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setFont(font1);

        horizontalLayout_10->addWidget(label_10);

        horizontalSlider_yaw = new QSlider(verticalLayoutWidget_3);
        horizontalSlider_yaw->setObjectName(QString::fromUtf8("horizontalSlider_yaw"));
        horizontalSlider_yaw->setMaximum(9999);
        horizontalSlider_yaw->setValue(5000);
        horizontalSlider_yaw->setOrientation(Qt::Horizontal);

        horizontalLayout_10->addWidget(horizontalSlider_yaw);

        lineEdit_yaw = new QLineEdit(verticalLayoutWidget_3);
        lineEdit_yaw->setObjectName(QString::fromUtf8("lineEdit_yaw"));
        lineEdit_yaw->setAlignment(Qt::AlignCenter);
        lineEdit_yaw->setClearButtonEnabled(true);

        horizontalLayout_10->addWidget(lineEdit_yaw);

        horizontalLayout_10->setStretch(0, 1);
        horizontalLayout_10->setStretch(1, 18);
        horizontalLayout_10->setStretch(2, 6);

        verticalLayout_2->addLayout(horizontalLayout_10);


        horizontalLayout_7->addLayout(verticalLayout_2);


        verticalLayout_5->addLayout(horizontalLayout_7);


        retranslateUi(AdjustPCClass);
        QObject::connect(lineEdit_x, SIGNAL(returnPressed()), AdjustPCClass, SLOT(on_lineEdit_x_returnPressed()));

        QMetaObject::connectSlotsByName(AdjustPCClass);
    } // setupUi

    void retranslateUi(QWidget *AdjustPCClass)
    {
        AdjustPCClass->setWindowTitle(QCoreApplication::translate("AdjustPCClass", "AdjustPC", nullptr));
        label_6->setText(QCoreApplication::translate("AdjustPCClass", "Translation:", nullptr));
        label_5->setText(QCoreApplication::translate("AdjustPCClass", "x: ", nullptr));
        label_4->setText(QCoreApplication::translate("AdjustPCClass", "y:", nullptr));
        label->setText(QCoreApplication::translate("AdjustPCClass", "z:", nullptr));
        label_7->setText(QCoreApplication::translate("AdjustPCClass", "Rotation:", nullptr));
        label_8->setText(QCoreApplication::translate("AdjustPCClass", "roll: ", nullptr));
        label_9->setText(QCoreApplication::translate("AdjustPCClass", "pitch:", nullptr));
        label_10->setText(QCoreApplication::translate("AdjustPCClass", "yaw:", nullptr));
    } // retranslateUi

};

namespace Ui {
    class AdjustPCClass: public Ui_AdjustPCClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ADJUSTPC_H
