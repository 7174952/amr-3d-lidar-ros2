#ifndef SUBWINDOW_FACELOGIN_H
#define SUBWINDOW_FACELOGIN_H

#include <QDialog>
#include <QCamera>
#include <QCameraViewfinder>
#include <QCameraImageCapture>
#include <QCameraInfo>
#include <QMessageBox>

#include "utils.h"
#include "global_dataset.h"

namespace Ui {
class SubWindow_FaceLogin;
}

class SubWindow_FaceLogin : public QDialog
{
    Q_OBJECT

public:
    explicit SubWindow_FaceLogin(QWidget *parent = nullptr);
    ~SubWindow_FaceLogin();
private slots:
    void onImageSaved(int id, const QString &fileName);

    void on_pushButton_saveImage_clicked();

private:
    Ui::SubWindow_FaceLogin *ui;
    QCamera *camera_;
    QCameraViewfinder *viewfinder_;
    QCameraImageCapture *imageCapture_;

};

#endif // SUBWINDOW_FACELOGIN_H
