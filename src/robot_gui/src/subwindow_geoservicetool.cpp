#include "subwindow_geoservicetool.h"
#include "ui_subwindow_geoservicetool.h"

SubWindow_GeoServiceTool::SubWindow_GeoServiceTool(rclcpp::Node::SharedPtr node, QWidget *parent) :
    QDialog(parent),
    node_(node),
    ui(new Ui::SubWindow_GeoServiceTool)
{
    ui->setupUi(this);

    geoTool = new GeoServiceTool(node_, ui->textEdit_geoShowMsg);
}

SubWindow_GeoServiceTool::~SubWindow_GeoServiceTool()
{
    delete ui;
}

void SubWindow_GeoServiceTool::onNewEnuYawUpdated(bool isValid, double yaw_rad)
{
    m_gnssPostion.yaw_offset = qRadiansToDegrees(yaw_rad);

    if(isValid && ui->pushButton_presetMapOrigin->isChecked())
    {
        m_mapOriginPostion.yaw_offset = qRadiansToDegrees(yaw_rad);
        QString text = QString("Map GNSS Origin")
                        + "\nlat:" + QString::number(m_mapOriginPostion.lat,'f',6)
                        + "\nlon:" + QString::number(m_mapOriginPostion.lon, 'f',6)
                        + "\nalt:" + QString::number(m_mapOriginPostion.alt, 'f',2)
                        + "\nyaw_offset(deg):" + QString::number(m_mapOriginPostion.yaw_offset,'f',6);

        ui->textEdit_mapOriginInfo->setPlainText(text);
        ui->pushButton_presetMapOrigin->setChecked(false);
    }
}

void SubWindow_GeoServiceTool::onNewFixReceived(int status, double conv, double lat, double lon, double alt)
{
    if((status < 0) || (conv > 0.05))
        return;
    m_gnssPostion.lat = lat;
    m_gnssPostion.lon = lon;
    m_gnssPostion.alt = alt;

}

void SubWindow_GeoServiceTool::on_pushButton_setDatum_clicked()
{
    geoTool->setDatum(ui->lineEdit_latitude->text().toDouble(),
                      ui->lineEdit_longitude->text().toDouble(),
                      ui->lineEdit_altitude->text().toDouble());
}


void SubWindow_GeoServiceTool::on_pushButton_fromLL_clicked()
{
    geoTool->fromLL(ui->lineEdit_fromLL_lat->text().toDouble(),
                    ui->lineEdit_fromLL_long->text().toDouble(),
                    ui->lineEdit_fromLL_alt->text().toDouble());

}


void SubWindow_GeoServiceTool::on_pushButton_presetMapOrigin_toggled(bool checked)
{
    if(checked)
    {
        //save origin gnss point (degree)
        m_mapOriginPostion.lat = m_gnssPostion.lat;
        m_mapOriginPostion.lon = m_gnssPostion.lon;
        m_mapOriginPostion.alt = m_gnssPostion.alt;
        ui->pushButton_presetMapOrigin->setText("Setting");
    }
    else
    {
        //save file
        QString path = Global_DataSet::instance().sysPath()["MapPath"] + "/" + Global_DataSet::instance().currentMapName() + "/gnss_map_origin.txt";
        QFile file(path);
        if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            qWarning() << "Can not open file:" << file.errorString();
            return;
        }
        QTextStream out(&file);
        out << "lat:" << QString::number(m_mapOriginPostion.lat,'f',6) << "\n"
            << "lon:" << QString::number(m_mapOriginPostion.lon,'f',6) << "\n"
            << "alt:" << QString::number(m_mapOriginPostion.alt,'f',2) << "\n"
            << "yaw_offset(deg):" << QString::number(m_mapOriginPostion.yaw_offset,'f',6) << "\n";
        file.close();
        ui->pushButton_presetMapOrigin->setText("Press to Setup");
    }
}


void SubWindow_GeoServiceTool::on_pushButton_getGnssOrigin_clicked()
{
    QVector3D currLL = geoTool->getCurrMapToLL();
    m_mapOriginPostion.lat = currLL.x();
    m_mapOriginPostion.lon = currLL.y();
    m_mapOriginPostion.alt = currLL.z();
    QString text = "lat:" + QString::number(currLL.x(),'f',6)
                 + "\nlon:" + QString::number(currLL.y(),'f',6)
                 + "\nalt:" + QString::number(currLL.z(),'f',2);
    ui->textEdit_gnssOrigin->setPlainText(text);

}


void SubWindow_GeoServiceTool::on_pushButton_gnssAheadPoint_clicked()
{
    QVector3D currLL = geoTool->getCurrMapToLL();
    m_aheadPointPostion.lat = currLL.x();
    m_aheadPointPostion.lon = currLL.y();
    m_aheadPointPostion.alt = currLL.z();

    QString text = "lat:" + QString::number(currLL.x(),'f',6)
                 + "\nlon:" + QString::number(currLL.y(),'f',6)
                 + "\nalt:" + QString::number(currLL.z(),'f',2);
    ui->textEdit_gnssAheadPoint->setPlainText(text);

}


void SubWindow_GeoServiceTool::on_pushButton_gnssEnuYaw_clicked()
{
    // 地球半径（米）
    const double R = 6378137.0;
    double dLat = qDegreesToRadians(m_aheadPointPostion.lat - m_mapOriginPostion.lat);
    double dLon = qDegreesToRadians(m_aheadPointPostion.lon - m_mapOriginPostion.lon);
    double latRefRad = qDegreesToRadians(m_mapOriginPostion.lat);

    double x_east = R * dLon * qCos(latRefRad);   // 东方向
    double y_north = R * dLat;                    // 北方向

    double angle_rad = qAtan2(y_north, x_east);  // atan2(Δ北, Δ东)
    double angle_deg = qRadiansToDegrees(angle_rad);
    angle_deg =  fmod(angle_deg + 360.0, 360.0);       // 归一化到 [0, 360)
    m_mapOriginPostion.yaw_offset = angle_deg;
    ui->lineEdit_gnssEnuYaw->setText(QString::number(angle_deg,'f',6));
}


void SubWindow_GeoServiceTool::on_pushButton_saveMapOriginFile_clicked()
{
    //save file
    QString path = Global_DataSet::instance().sysPath()["MapPath"] + "/" + Global_DataSet::instance().currentMapName() + "/gnss_map_origin.txt";
    QFile file(path);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qWarning() << "Can not open file:" << file.errorString();
        return;
    }
    QTextStream out(&file);
    out << "lat:" << QString::number(m_mapOriginPostion.lat,'f',6) << "\n"
        << "lon:" << QString::number(m_mapOriginPostion.lon,'f',6) << "\n"
        << "alt:" << QString::number(m_mapOriginPostion.alt,'f',2) << "\n"
        << "yaw_offset(deg):" << QString::number(m_mapOriginPostion.yaw_offset,'f',6) << "\n";
    file.close();
    ui->textEdit_geoShowMsg->append("Save File Completed");
}

