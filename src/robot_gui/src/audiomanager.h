#ifndef AUDIOMANAGER_H
#define AUDIOMANAGER_H

#include <QObject>
#include <QMediaPlayer>
#include <QTimer>

class AudioManager : public QObject
{
    Q_OBJECT

public:
    explicit AudioManager(QObject *parent = nullptr);

    void setGreetAudio(const QString &path);
    void setObstacleAudio(const QString &path);
    void setGuideAudio(const QString &path);

    void setLoopIntervalForObstacle(int ms);
    void setLoopIntervalForGuide(int ms);

    void playGreet();
    void startObstacle();
    void stopObstacle();
    void startGuide();
    void stopGuide();

signals:
    void greetFinished();

private:
    QMediaPlayer *playerGreet;
    QMediaPlayer *playerObstacle;
    QMediaPlayer *playerGuide;

    QTimer *obstacleLoopTimer;
    QTimer *guideLoopTimer;

    QString greetPath;
    QString obstaclePath;
    QString guidePath;

    bool obstacleLooping = false;
    bool guideLooping = false;
};

#endif // AUDIOMANAGER_H
