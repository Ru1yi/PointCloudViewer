#include "AdjustPC.h"

AdjustPC::AdjustPC(QWidget* parent)
	: QWidget(parent)
{
	ui.setupUi(this);
}

void AdjustPC::on_lineEdit_returnPressed()
{
	emit sendSignal_lineEdit_returnPressed(this->ui.lineEdit_x->text(),
										    this->ui.lineEdit_y->text(),
										    this->ui.lineEdit_z->text(),
											this->ui.lineEdit_roll->text(),
											this->ui.lineEdit_pitch->text(),
											this->ui.lineEdit_yaw->text());
}

AdjustPC::~AdjustPC()
{}


