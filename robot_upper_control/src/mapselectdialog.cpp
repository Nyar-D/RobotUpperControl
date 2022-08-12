#include "mapselectdialog.h"

MapSelectDialog::MapSelectDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::MapSelectDialog)
{
  ui->setupUi(this);
}

MapSelectDialog::~MapSelectDialog()
{
  delete ui;
}
