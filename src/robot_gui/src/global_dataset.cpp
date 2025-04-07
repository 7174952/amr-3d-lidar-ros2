#include "global_dataset.h"

Global_DataSet& Global_DataSet::instance()
{
    static Global_DataSet instance;
    return instance;
}

Global_DataSet::Global_DataSet()
{
    m_mapName = "";
}

Global_DataSet::~Global_DataSet()
{

}

void Global_DataSet::setSysPath(const QMap<QString, QString>& sys_path)
{
    m_path = sys_path;
}

QMap<QString, QString> Global_DataSet::sysPath() const
{
    return m_path;
}

void Global_DataSet::setCurrentMapName(const QString& map_name)
{
    m_mapName = map_name;
}

QString Global_DataSet::currentMapName() const
{
    return m_mapName;
}

void Global_DataSet::setRobotSize(const QMap<QString, QString>& robot_size)
{
    m_robotSize = robot_size;
}

QMap<QString, QString> Global_DataSet::robotSize() const
{
    return m_robotSize;
}

void Global_DataSet::setDebugMode(const bool& debug_mode)
{
    m_debugMode = debug_mode;
}

bool Global_DataSet::debugMode() const
{
    return m_debugMode;
}

void Global_DataSet::setSensorEnable(const QString& name, bool enable)
{
    m_sensorEnable[name] = enable;
}

bool Global_DataSet::sensorEnable(const QString& name) const
{
    return m_sensorEnable[name];
}

void Global_DataSet::setGuideFunctionEn(const bool enable)
{
    m_guideFunctionEn = enable;
}

bool Global_DataSet::guideFunctionEn() const
{
    return m_guideFunctionEn;
}
