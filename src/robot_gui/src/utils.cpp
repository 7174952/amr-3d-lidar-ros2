#include "utils.h"

void Utils::start_process(QProcess* rosProcess, QString package, QString fileName)
{    
    rosProcess->start("bash", QStringList() << "-c" << "exec setsid ros2 launch " + package + " " + fileName, QIODevice::ReadWrite);
    rosProcess->waitForStarted();
}

void Utils::terminate_process(QProcess* rosProcess)
{
    if(rosProcess && (rosProcess->state() != QProcess::NotRunning))
    {
        pid_t pid = rosProcess->processId();
        // 发送 SIGTERM 信号给进程组（负的 pid 表示整个组）
        kill(-pid, SIGTERM);

        if(!rosProcess->waitForFinished(3000))
        {
            kill(-pid, SIGKILL);
            rosProcess->waitForFinished();
        }
    }
}

QString Utils::execute_shell_cmd(QString shell)
{
    QProcess proc;
    proc.start("bash", QStringList() << shell);
    if(!proc.waitForFinished(10000))
    {
        return "timeout";
    }

    QString Result = proc.readAllStandardOutput();
    return Result;
}
