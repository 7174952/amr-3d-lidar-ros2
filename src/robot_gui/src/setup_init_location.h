#ifndef SETUP_INIT_LOCATION_H
#define SETUP_INIT_LOCATION_H

#include <QDialog>
#include <QMap>
#include <QMessageBox>

#include "global_dataset.h"
#include "utils.h"

namespace Ui {
class Setup_Init_Location;
}

class Setup_Init_Location : public QDialog
{
    Q_OBJECT

public:
    explicit Setup_Init_Location(QWidget *parent = nullptr);
    ~Setup_Init_Location();

public slots:
    void onOdomReceived(QVector<double> odom_pose);
    void updateMapName(const QString& newMapName);

private slots:
    void on_pushButton_SetLocation_clicked();

private:
    Ui::Setup_Init_Location *ui;
    QString m_mapName;

    //define pose
    struct Robot_Pose
    {
        double pos_x;
        double pos_y;
        double pos_z;
        double ori_x;
        double ori_y;
        double ori_z;
        double ori_w;
    };
    Robot_Pose robot_cur_pose;
    QMap<QString, Robot_Pose> init_location_list;

};

#endif // SETUP_INIT_LOCATION_H
