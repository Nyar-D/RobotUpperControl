#include <ros/ros.h>
#include <QStringList>
#include "mapselectdialog.h"

#include <QDebug>

MapSelectDialog::MapSelectDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::MapSelectDialog)
{
  ui->setupUi(this);

  // Tool已激活
  enableTool(true);

  ros::NodeHandle nh("map_config");
  std::string cur_map;
  std::vector<std::string> map_list;

  if (nh.getParam("current_map", cur_map) && nh.getParam("map_list", map_list)) {
    for (auto iter = map_list.begin(); iter != map_list.end(); iter++) {
      QListWidgetItem *item = new QListWidgetItem(ui->mapList_lw);
      item->setText(QString::fromStdString(*iter));
      ui->mapList_lw->addItem(item);

      if (*iter == cur_map)
        ui->mapList_lw->setCurrentItem(item);
    }
  } else {
    ui->mapList_lw->addItem(tr("当前无地图发布！"));
  }
}

MapSelectDialog::~MapSelectDialog()
{
  delete ui;
}

void MapSelectDialog::accept()
{
  QString genMapName = ui->mapList_lw->currentItem()->text();
  qDebug() << "选择地图:" << genMapName;
  if (isToolActive) {
    emit confirm_signal(genMapName);
    emit dialogClose();
 }

  QDialog::accept();
}

void MapSelectDialog::reject()
{
  if (isToolActive)
    emit dialogClose();

  QDialog::reject();
}

void MapSelectDialog::closeDialog(void)
{
    // Tool已关闭
    enableTool(false);
    this->close();
}

void MapSelectDialog::enableTool(bool flag)
{
  isToolActive = flag;
}

void MapSelectDialog::on_newMap_btn_clicked()
{
  QListWidgetItem *item = new QListWidgetItem("new map", ui->mapList_lw);
  ui->mapList_lw->addItem(item);
  ui->mapList_lw->setCurrentItem(item);
  item->setFlags(Qt::ItemIsEnabled|Qt::ItemIsEditable);
}

