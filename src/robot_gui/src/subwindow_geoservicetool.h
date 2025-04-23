#ifndef SUBWINDOW_GEOSERVICETOOL_H
#define SUBWINDOW_GEOSERVICETOOL_H

#include <QDialog>
#include <QFile>
#include <QTextStream>
#include <QVector3D>
#include <QtMath>

#include <rclcpp/rclcpp.hpp>

#include "utils.h"
#include "global_dataset.h"

namespace Ui {
class SubWindow_GeoServiceTool;
}

class SubWindow_GeoServiceTool : public QDialog
{
    Q_OBJECT

public:
    explicit SubWindow_GeoServiceTool(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
    ~SubWindow_GeoServiceTool();

public slots:
    void onNewEnuYawUpdated(bool isValid, double yaw_rad);
    void onNewFixReceived(int status, double conv, double lat, double lon, double alt);

private slots:
    void on_pushButton_setDatum_clicked();

    void on_pushButton_fromLL_clicked();
    void on_pushButton_presetMapOrigin_toggled(bool checked);

    void on_pushButton_getGnssOrigin_clicked();

    void on_pushButton_gnssAheadPoint_clicked();

    void on_pushButton_gnssEnuYaw_clicked();

    void on_pushButton_saveMapOriginFile_clicked();

private:
    struct GnssPostion
    {
        double lat;
        double lon;
        double alt;
        double yaw_offset;
    };
    GnssPostion m_gnssPostion;
    GnssPostion m_mapOriginPostion;    
    GnssPostion m_aheadPointPostion;


private:
    Ui::SubWindow_GeoServiceTool *ui;
    rclcpp::Node::SharedPtr node_;
    GeoServiceTool *geoTool;
};

#endif // SUBWINDOW_GEOSERVICETOOL_H
