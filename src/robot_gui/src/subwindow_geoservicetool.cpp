#include "subwindow_geoservicetool.h"
#include "ui_subwindow_geoservicetool.h"

SubWindow_GeoServiceTool::SubWindow_GeoServiceTool(rclcpp::Node::SharedPtr node, QWidget *parent) :
    QDialog(parent),
    node_(node),
    ui(new Ui::SubWindow_GeoServiceTool)
{
    ui->setupUi(this);

    geoTool = new GeoServiceTool(node_, ui->textEdit_geoShowMsg);
    gpsConvThreshold = Global_DataSet::instance().gnssNtrip()["ConvThreshold"].toDouble();
    is_gnssValid = false;

}

SubWindow_GeoServiceTool::~SubWindow_GeoServiceTool()
{
    delete ui;
}

void SubWindow_GeoServiceTool::onNewFixReceived(int status, double conv, double lat, double lon, double alt)
{
    gpsConvThreshold = Global_DataSet::instance().gnssNtrip()["ConvThreshold"].toDouble();
    if(status < 0)
    {
        return;
    }

    is_gnssValid = (conv > gpsConvThreshold) ? false : true;
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

void SubWindow_GeoServiceTool::on_pushButton_getGnssOrigin_clicked()
{
    if(is_gnssValid)
    {
        m_mapOriginPostion.lat = m_gnssPostion.lat;
        m_mapOriginPostion.lon = m_gnssPostion.lon;
        m_mapOriginPostion.alt = m_gnssPostion.alt;
    }
    else
    {
        QVector3D currLL = geoTool->getCurrMapToLL();
        m_mapOriginPostion.lat = currLL.x();
        m_mapOriginPostion.lon = currLL.y();
        m_mapOriginPostion.alt = currLL.z();
    }

    QString text = "lat:" + QString::number(m_mapOriginPostion.lat,'f',6)
                 + "\nlon:" + QString::number(m_mapOriginPostion.lon,'f',6)
                 + "\nalt:" + QString::number(m_mapOriginPostion.alt,'f',2);
    ui->textEdit_gnssOrigin->setPlainText(text);

}


void SubWindow_GeoServiceTool::on_pushButton_gnssAheadPoint_clicked()
{
    if(is_gnssValid)
    {
        m_aheadPointPostion.lat = m_gnssPostion.lat;
        m_aheadPointPostion.lon = m_gnssPostion.lon;
        m_aheadPointPostion.alt = m_gnssPostion.alt;
    }
    else
    {
        QVector3D currLL = geoTool->getCurrMapToLL();
        m_aheadPointPostion.lat = currLL.x();
        m_aheadPointPostion.lon = currLL.y();
        m_aheadPointPostion.alt = currLL.z();
    }

    QString text = "lat:" + QString::number(m_aheadPointPostion.lat,'f',6)
                 + "\nlon:" + QString::number(m_aheadPointPostion.lon,'f',6)
                 + "\nalt:" + QString::number(m_aheadPointPostion.alt,'f',2);
    ui->textEdit_gnssAheadPoint->setPlainText(text);

}


void SubWindow_GeoServiceTool::on_pushButton_gnssEnuYaw_clicked()
{
    m_mapOriginPostion.yaw_offset = geoTool->getGnssMoveOrientalDeg(m_aheadPointPostion, m_mapOriginPostion);
    ui->lineEdit_gnssEnuYaw->setText(QString::number(m_mapOriginPostion.yaw_offset,'f',6));
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


void SubWindow_GeoServiceTool::on_pushButton_getCurrGnssPos_clicked()
{

    m_mapOriginPostion.lat = m_gnssPostion.lat;
    m_mapOriginPostion.lon = m_gnssPostion.lon;
    m_mapOriginPostion.alt = m_gnssPostion.alt;

    ui->lineEdit_latitude->setText(QString::number(m_mapOriginPostion.lat,'f',6));
    ui->lineEdit_longitude->setText(QString::number(m_mapOriginPostion.lon,'f',6));
    ui->lineEdit_altitude->setText(QString::number(m_mapOriginPostion.alt,'f',2));


}

