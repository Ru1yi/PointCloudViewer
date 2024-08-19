#include "AdjustPC.h"

AdjustPC::AdjustPC(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
}

void AdjustPC::on_lineEdit_x_returnPressed()
{
	sendSignal_lineEdit_x_returnPressed(this->ui.lineEdit_x->text());
}

AdjustPC::~AdjustPC()
{}


