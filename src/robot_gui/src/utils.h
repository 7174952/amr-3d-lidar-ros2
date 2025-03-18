#ifndef UTILS_H
#define UTILS_H

#include <QString>
#include <QProcess>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <QDebug>

namespace Ui {
    class Utils;
}

class Utils
{
public:
    static void start_process(QProcess *, QString, QString);
    static void terminate_process(QProcess *);
    static QString execute_shell_cmd(QString);


private:
    Utils() = delete;
    ~Utils() = delete;

};

#endif // UTILS_H
