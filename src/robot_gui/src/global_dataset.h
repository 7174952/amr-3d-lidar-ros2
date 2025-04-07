#ifndef GLOBAL_DATASET_H
#define GLOBAL_DATASET_H
#include <QString>
#include <QMap>

namespace Ui {
    class Global_DataSet;
}

class Global_DataSet
{
public:
    static Global_DataSet& instance();

    //setter and getter
    void setSysPath(const QMap<QString, QString>& sys_path);
    QMap<QString, QString> sysPath() const;

    void setRobotSize(const QMap<QString, QString>& robot_size);
    QMap<QString, QString> robotSize() const;

    void setCurrentMapName(const QString& map_name);
    QString currentMapName() const;

    void setDebugMode(const bool& debug_mode);
    bool debugMode() const;

    void setSensorEnable(const QString& name, bool enable);
    bool sensorEnable(const QString& name) const;

    void setGuideFunctionEn(const bool enable);
    bool guideFunctionEn() const;


private:
    Global_DataSet();   // 私有构造函数
    ~Global_DataSet();  // 私有析构函数

    // 禁止拷贝与赋值
    Global_DataSet(const Global_DataSet&) = delete;
    Global_DataSet& operator=(const Global_DataSet&) = delete;

    // 成员变量
    QMap<QString, QString> m_path;
    QString m_mapName;
    QMap<QString, QString> m_robotSize;
    bool m_debugMode;
    QMap<QString, bool> m_sensorEnable;
    bool m_guideFunctionEn;


};

#endif // GLOBAL_DATASET_H
