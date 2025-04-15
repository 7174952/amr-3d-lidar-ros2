#ifndef SUBWINDOW_GUIDEROBOT_H
#define SUBWINDOW_GUIDEROBOT_H

#include <QDialog>
#include <QProcess>
#include <QFile>
#include <QDir>
#include <QMessageBox>
#include <QRegExp>
#include <QMap>
#include <QtGlobal>
#include <QStyledItemDelegate>
#include <QStandardItemModel>
#include <QListView>
#include <QPainter>
#include <QCheckBox>
#include <QApplication>
#include <QMouseEvent>
#include <QSortFilterProxyModel>
#include <QtGui/QVector3D>
#include <cmath>
#include <QSettings>

//ros2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include "utils.h"
#include "global_dataset.h"
#include "audiomanager.h"

namespace Ui {
class SubWindow_GuideRobot;
class ButtonCheckDelegate;
class FilterProxy;
}

class ButtonCheckDelegate : public QStyledItemDelegate
{
public:
    void paint(QPainter *painter, const QStyleOptionViewItem &option,
               const QModelIndex &index) const override
    {
        painter->save();

        QRect rect = option.rect;
        QString text = index.data(Qt::DisplayRole).toString();
        Qt::CheckState checkState = static_cast<Qt::CheckState>(index.data(Qt::CheckStateRole).toInt());

        // 主按钮样式绘制
        QColor bgColor = (option.state & QStyle::State_Selected) ? QColor("#aed6f1") : QColor("#f5f5f5");
        QRect buttonRect = rect.adjusted(2, 2, -160, -2);
        painter->setBrush(bgColor);
        painter->setPen(QColor("#2980b9"));
        painter->drawRoundedRect(buttonRect, 6, 6);
        painter->setPen(Qt::black);
        painter->drawText(buttonRect, Qt::AlignCenter, text);

        // CheckBox 区域
        QRect checkRect = QRect(rect.right() - 140, rect.center().y() - 10, 20, 20);
        QStyleOptionButton checkOpt;
        checkOpt.state = QStyle::State_Enabled |
            (checkState == Qt::Checked ? QStyle::State_On : QStyle::State_Off);
        checkOpt.rect = checkRect;

        QApplication::style()->drawControl(QStyle::CE_CheckBox, &checkOpt, painter);

        // CheckBox 后面的文字
        QRect labelRect = QRect(checkRect.right() + 2, checkRect.top(), 100, 20);
        QString checkLabel = "Turn Back";
        painter->drawText(labelRect, Qt::AlignVCenter | Qt::AlignLeft, checkLabel);

        painter->restore();
    }

    bool editorEvent(QEvent *event, QAbstractItemModel *model,
                     const QStyleOptionViewItem &option, const QModelIndex &index) override
    {
        if (event->type() == QEvent::MouseButtonRelease)
        {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            QRect rect = option.rect;
            QRect checkRect = QRect(rect.right() - 140, rect.center().y() - 10, 20, 20);
            if (checkRect.contains(mouseEvent->pos()))
            {
                // 切换勾选状态
                Qt::CheckState state = static_cast<Qt::CheckState>(
                    index.data(Qt::CheckStateRole).toInt());
                model->setData(index, state == Qt::Checked ? Qt::Unchecked : Qt::Checked, Qt::CheckStateRole);
                return true;
            }
        }
        return QStyledItemDelegate::editorEvent(event, model, option, index);
    }

    QSize sizeHint(const QStyleOptionViewItem &, const QModelIndex &) const override
    {
        return QSize(100, 40);
    }
};

// 自定义过滤器：根据关键词过滤显示条目
class FilterProxy : public QSortFilterProxyModel
{
public:
    void setKeyword(const QString &kw)
    {
        keyword = kw;
        invalidateFilter();
    }

protected:
    bool filterAcceptsRow(int row, const QModelIndex &parent) const override
    {
        QModelIndex index = sourceModel()->index(row, 0, parent);
        QString text = index.data(Qt::DisplayRole).toString();
        return keyword.isEmpty() || text.contains(keyword, Qt::CaseInsensitive);
    }

private:
    QString keyword;
};

class SubWindow_GuideRobot : public QDialog
{
    Q_OBJECT

public:
    explicit SubWindow_GuideRobot(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
    ~SubWindow_GuideRobot();

protected:
    void closeEvent(QCloseEvent *event) override
    {
        emit subWindowClosed();
        QDialog::closeEvent(event);
    }

signals:
    void subWindowClosed();
    void odomReceived();
    void sendMessage(const QString &message); //send to mainwindow statusBar

public slots:
     void updateMapName(const QString& newMapName);
     void onGuideCameraStateChanged(const int arg);
     void onGreetVoiceFinished();
#if 1 //debug_ryu
     void handleOutput();
     void handleError();
#endif

private slots:
    void on_pushButton_startRobot_toggled(bool checked);

    void on_pushButton_NaviGo_toggled(bool checked);
    void onItemClicked(const QModelIndex &proxyIndex);
    void onSearchCurrLocation();
    void onSetNaviRules();

    void on_checkBox_cameraGuide_stateChanged(int arg1);

private:
    void Odometry_CallBack(const nav_msgs::msg::Odometry& odom);
    void reachedGoal_CallBack(const std_msgs::msg::Bool& msg);
    void personDetect_CallBack(const std_msgs::msg::String& msg);
    void obstacle_CallBack(const std_msgs::msg::Int32& msg);
    void chatbot_state_CallBack(const std_msgs::msg::String& msg);
    void upload_RouteList();
    void upload_LocationList();
    void saveConfig();
    void initConfig();
    void wakeupChatbot(bool req);


private:
    rclcpp::Node::SharedPtr node_;
    QString m_mapName;
    QProcess* guide_robot_process;
    QVector<double> last_point;
    QProcess* detect_process;
    QProcess* chatbot_process;

    //define location
    struct Robot_Position
    {
        double x;
        double y;
        double z;
    };
    QMap<QString, Robot_Position> location_list;
    QString current_location;
    QString navi_start;
    QString navi_target;
    double final_direct;
    bool reached_goal;

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
    Robot_Pose init_pose;

    //define route
    struct Rule
    {
        Robot_Position pos_in;  // begin
        Robot_Position pos_out; // end
        //rules
        double max_vel;
        double width_tolerance;
    };
    struct Route_Info
    {
        QString start;
        QString target;
        double distance;
        QList<Rule> rules_list;

        void clear()
        {
            start.clear();
            target.clear();
            distance = 0.0;
            rules_list.clear();
        }
    };
    QMap<QString, Route_Info> route_list;
    nav_msgs::msg::Path waypoints_msg;

    //person detect
    struct Person_Info
    {
        uint last_num;
        uint curr_num;
        uint cnt;
        double near_distance;
        const double FAR_START = 3.5;
        const double FAR_STOP = 4.0;
        const uint DETECT_MAX_TIMES = 5;
        bool guide_en;
        double speed_rate;  //0.0~1.0, 0.0-stop, 1.0-max vel
    };
    Person_Info person_info;

    struct Rule_State
    {
        int cnt;
        QString status; //"in","out"
    };
    Rule_State rule_state;
    Route_Info rules_route;

    QStandardItemModel *baseModel;
    FilterProxy *proxyModel;

    AudioManager *audioManager;
    QMap<QString, QString> system_path;
    int32_t obstacle_points_num;
    bool guide_annouse;
    bool is_greet;

    //chatbot
    QString chatbot_state;

private:
    Ui::SubWindow_GuideRobot *ui;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rules_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr guide_pub;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_goal_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr person_detect_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr obstacle_num_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chatbot_state_sub;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr chatbot_request;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_pub;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client;

    int idx_;


};


#endif // SUBWINDOW_GUIDEROBOT_H
