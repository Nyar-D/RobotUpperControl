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
  void accept();
  void reject();
  void enableTool(bool);

private slots:
  void closeDialog(void);
  void on_newMap_btn_clicked();

signals:
  void confirm_signal(QString mapName);
  void dialogClose(void);

private:
  Ui::MapSelectDialog *ui;
  int isToolActive = 0;
};

#endif // MAPSELECTDIALOG_H
