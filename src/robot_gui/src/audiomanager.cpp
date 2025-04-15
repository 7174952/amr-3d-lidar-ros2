// AudioManager.cpp
#include "audiomanager.h"
#include <QUrl>
#include <QDebug>

AudioManager::AudioManager(QObject *parent) : QObject(parent)
{
    // 初始化播放器
    playerGreet = new QMediaPlayer(this);
    playerGreet->setVolume(100);
    connect(playerGreet, &QMediaPlayer::mediaStatusChanged, this, [=](QMediaPlayer::MediaStatus status)
    {
        if (status == QMediaPlayer::EndOfMedia)
        {
            emit greetFinished();
        }
    });

    playerObstacle = new QMediaPlayer(this);
    playerObstacle->setVolume(100);
    connect(playerObstacle, &QMediaPlayer::mediaStatusChanged, this, [=](QMediaPlayer::MediaStatus status)
    {
        if (status == QMediaPlayer::EndOfMedia && obstacleLooping)
        {
            obstacleLoopTimer->start();
        }
    });

    playerGuide = new QMediaPlayer(this);
    playerGuide->setVolume(100);
    connect(playerGuide, &QMediaPlayer::mediaStatusChanged, this, [=](QMediaPlayer::MediaStatus status)
    {
        if (status == QMediaPlayer::EndOfMedia && guideLooping)
        {
            guideLoopTimer->start();
        }
    });

    // 循环播放定时器
    obstacleLoopTimer = new QTimer(this);
    obstacleLoopTimer->setInterval(3000);
    obstacleLoopTimer->setSingleShot(true);
    connect(obstacleLoopTimer, &QTimer::timeout, this, [=]()
    {
        if (obstacleLooping && playerGuide->state() != QMediaPlayer::PlayingState)
        {
            playerObstacle->play();
        }
    });

    guideLoopTimer = new QTimer(this);
    guideLoopTimer->setInterval(3000);
    guideLoopTimer->setSingleShot(true);
    connect(guideLoopTimer, &QTimer::timeout, this, [=]()
    {
        if (guideLooping)
        {
            playerGuide->play();
        }
    });
}

void AudioManager::setGreetAudio(const QString &path)
{
    greetPath = path;
    playerGreet->setMedia(QUrl::fromLocalFile(greetPath));
}

void AudioManager::setObstacleAudio(const QString &path)
{
    obstaclePath = path;
    playerObstacle->setMedia(QUrl::fromLocalFile(obstaclePath));
}

void AudioManager::setGuideAudio(const QString &path)
{
    guidePath = path;
    playerGuide->setMedia(QUrl::fromLocalFile(guidePath));
}

void AudioManager::setLoopIntervalForObstacle(int ms)
{
    obstacleLoopTimer->setInterval(ms);
}

void AudioManager::setLoopIntervalForGuide(int ms)
{
    guideLoopTimer->setInterval(ms);
}

void AudioManager::playGreet()
{
    if (greetPath.isEmpty())
    {
        qWarning() << "greet 路径未设置";
        return;
    }
    playerGreet->stop();
    playerGreet->play();
}

void AudioManager::stopGreet()
{
    playerGreet->stop();
}

void AudioManager::startObstacle()
{
    if (obstaclePath.isEmpty()) return;
    if (playerGuide->state() == QMediaPlayer::PlayingState) return;
    if (obstacleLooping) return;

    obstacleLooping = true;
    playerObstacle->stop();
    playerObstacle->play();
}

void AudioManager::stopObstacle()
{
    obstacleLooping = false;
    obstacleLoopTimer->stop();
    playerObstacle->stop();
}

void AudioManager::startGuide()
{
    if (guidePath.isEmpty()) return;
    if (guideLooping) return;

    stopObstacle();
    guideLooping = true;
    playerGuide->stop();
    playerGuide->play();
}

void AudioManager::stopGuide()
{
    guideLooping = false;
    guideLoopTimer->stop();
    playerGuide->stop();
}
