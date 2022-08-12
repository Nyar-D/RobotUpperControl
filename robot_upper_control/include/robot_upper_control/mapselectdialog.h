#ifndef MAPSELECTDIALOG_H
#define MAPSELECTDIALOG_H

#include <QDialog>
#include "ui_mapselectdialog.h"

namespace Ui {
class MapSelectDialog;
}

class MapSelectDialog : public QDialog
{
  Q_OBJECT

public:
  explicit MapSelectDialog(QWidget *parent = nullptr);
  ~MapSelectDialog();

private:
  Ui::MapSelectDialog *ui;
};

#endif // MAPSELECTDIALOG_H
