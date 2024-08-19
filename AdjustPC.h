#pragma once

#include <QWidget>
#include <QDebug>
#include "ui_AdjustPC.h"

class AdjustPC : public QWidget
{
	Q_OBJECT

public:
	AdjustPC(QWidget *parent = nullptr);
	~AdjustPC();

signals:
	void sendSignal_lineEdit_returnPressed(QString, QString, QString, QString, QString, QString);

private:
	Ui::AdjustPCClass ui;

private slots:
	void on_lineEdit_returnPressed();
};
