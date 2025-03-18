#ifndef SUBWINDOW_SYSTEM_H
#define SUBWINDOW_SYSTEM_H

#include <QDialog>
#include <QFileDialog>
#include <QDir>
#include <QMap>
#include <QSettings>

namespace Ui {
class SubWindow_System;
}

class SubWindow_System : public QDialog
{
    Q_OBJECT

public:
    explicit SubWindow_System(QWidget *parent = nullptr);
    ~SubWindow_System();

private:
    void initConfig();
    void saveConfig();

private slots:
    void on_pushButton_ProjectPath_clicked();

    void on_pushButton_InstallPath_clicked();

    void on_pushButton_MapPath_clicked();

    void on_pushButton_PointCloudPath_clicked();

    void on_pushButton_BagDataPath_clicked();

    void on_pushButton_ScriptPath_clicked();

    void on_pushButton_LaunchPath_clicked();

    void on_pushButton_ResourcePath_clicked();

    void on_buttonBox_accepted();

private:
    QString setup_path();

public:
    QMap<QString, QString> getSystemConfigPath();

private:
    Ui::SubWindow_System *ui;
    QMap<QString, QString> system_path;
};

#endif // SUBWINDOW_SYSTEM_H
