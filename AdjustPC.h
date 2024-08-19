#pragma once

#include <QWidget>
#include "ui_AdjustPC.h"

class AdjustPC : public QWidget
{
	Q_OBJECT

public:
	AdjustPC(QWidget *parent = nullptr);
	~AdjustPC();

signals:
	void sendSignal_lineEdit_x_returnPressed(QString);

private:
	Ui::AdjustPCClass ui;

private slots:
	void on_lineEdit_x_returnPressed();
};
