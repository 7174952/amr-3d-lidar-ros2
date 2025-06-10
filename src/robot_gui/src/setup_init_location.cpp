#include "setup_init_location.h"
#include "ui_setup_init_location.h"

Setup_Init_Location::Setup_Init_Location(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Setup_Init_Location)
{
    ui->setupUi(this);
}

Setup_Init_Location::~Setup_Init_Location()
{
    delete ui;
}

void Setup_Init_Location::updateMapName(const QString& newMapName)
{
    m_mapName = newMapName;

    QString filePath = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName + "/init_location_list.txt";
    QFile file(filePath);
    if(file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QTextStream in(&file);

        while (!in.atEnd())
        {
            QStringList line = in.readLine().trimmed().split(":");
            QStringList strVal = line.at(1).split(",");

            init_location_list[line.at(0)].pos_x = strVal.at(0).toDouble();
            init_location_list[line.at(0)].pos_y = strVal.at(1).toDouble();
            init_location_list[line.at(0)].pos_z = strVal.at(2).toDouble();
            init_location_list[line.at(0)].ori_x = strVal.at(3).toDouble();
            init_location_list[line.at(0)].ori_y = strVal.at(4).toDouble();
            init_location_list[line.at(0)].ori_z = strVal.at(5).toDouble();
            init_location_list[line.at(0)].ori_w = strVal.at(6).toDouble();
            ui->textEdit_showStatus->append(line.at(0) + ":" + strVal.at(0) + "," + strVal.at(1) + "," + strVal.at(2));
        }
        file.close();
    }

}


void Setup_Init_Location::onOdomReceived(QVector<double> odom_pose)
{
    robot_cur_pose.pos_x = odom_pose.at(0);
    robot_cur_pose.pos_y = odom_pose.at(1);
    robot_cur_pose.pos_z = odom_pose.at(2);
    robot_cur_pose.ori_x = odom_pose.at(3);
    robot_cur_pose.ori_y = odom_pose.at(4);
    robot_cur_pose.ori_z = odom_pose.at(5);
    robot_cur_pose.ori_w = odom_pose.at(6);
}

void Setup_Init_Location::on_pushButton_SetLocation_clicked()
{

    QString filePath = Global_DataSet::instance().sysPath()["MapPath"] + "/" + m_mapName + "/init_location_list.txt";
    QFile file(filePath);

    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QMessageBox::warning(this,"Warning","Open init_location_list.txt file failed!");
    }

    QTextStream out(&file);
    init_location_list[ui->lineEdit_name->text()].pos_x = robot_cur_pose.pos_x;
    init_location_list[ui->lineEdit_name->text()].pos_y = robot_cur_pose.pos_y;
    init_location_list[ui->lineEdit_name->text()].pos_z = robot_cur_pose.pos_z;
    init_location_list[ui->lineEdit_name->text()].ori_x = robot_cur_pose.ori_x;
    init_location_list[ui->lineEdit_name->text()].ori_y = robot_cur_pose.ori_y;
    init_location_list[ui->lineEdit_name->text()].ori_z = robot_cur_pose.ori_z;
    init_location_list[ui->lineEdit_name->text()].ori_w = robot_cur_pose.ori_w;
    for(auto it = init_location_list.begin(); it != init_location_list.end(); it++)
    {
        out << it.key() << ":"
            << QString::number(it.value().pos_x,'f',6) << ","
            << QString::number(it.value().pos_y,'f',6) << ","
            << QString::number(it.value().pos_z,'f',6) << ","
            << QString::number(it.value().ori_x,'f',6) << ","
            << QString::number(it.value().ori_y,'f',6) << ","
            << QString::number(it.value().ori_z,'f',6) << ","
            << QString::number(it.value().ori_w,'f',6) << "\n";
    }
    ui->textEdit_showStatus->append(ui->lineEdit_name->text() + ":" + QString::number(robot_cur_pose.pos_x,'f',6) + ","
                                    + QString::number(robot_cur_pose.pos_y,'f',6) + ","
                                    + QString::number(robot_cur_pose.pos_z,'f',6));

    file.close();

}

