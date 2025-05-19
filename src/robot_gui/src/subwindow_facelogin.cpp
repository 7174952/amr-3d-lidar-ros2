#include "subwindow_facelogin.h"
#include "ui_subwindow_facelogin.h"

SubWindow_FaceLogin::SubWindow_FaceLogin(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SubWindow_FaceLogin)
{
    ui->setupUi(this);

    camera_ = nullptr;
    QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
    if (!cameras.isEmpty())
    {
        for(const QCameraInfo &cameraInfo : cameras)
        {
            if(cameraInfo.description().contains("USB CAMERA"))
            {
                camera_ = new QCamera(cameraInfo, this);
                break;
            }
        }
    }

    if(!camera_)
    {
        camera_ = new QCamera(QCamera::availableDevices().first(), this);
    }
    viewfinder_ = new QCameraViewfinder(this);
    imageCapture_ = new QCameraImageCapture(camera_, this);

    ui->verticalLayout->addWidget(viewfinder_);
    camera_->setViewfinder(viewfinder_);
    camera_->start();

    connect(imageCapture_, &QCameraImageCapture::imageSaved, this, &SubWindow_FaceLogin::onImageSaved);

    //list all user folder
    QString path = Global_DataSet::instance().sysPath()["ScriptPath"] + "/face_detect/TARGETIMG";
    QDir dir(path);
    if(!dir.exists())
    {
        QMessageBox::warning(this,"Warning", path + " Not exist!");
    }
    else
    {
        dir.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);
        QFileInfoList folderList = dir.entryInfoList();
        ui->comboBox_userFolder->clear();
        for(const QFileInfo &info : folderList)
        {
            ui->comboBox_userFolder->addItem(info.fileName());
        }
    }

}

SubWindow_FaceLogin::~SubWindow_FaceLogin()
{
    camera_->stop();
    delete ui;
}

void SubWindow_FaceLogin::onImageSaved(int id, const QString &fileName)
{
    Q_UNUSED(id)
    qDebug() << "图片已保存至:" << fileName;
}

void SubWindow_FaceLogin::on_pushButton_saveImage_clicked()
{
    QString path = Global_DataSet::instance().sysPath()["ScriptPath"] + "/face_detect/TARGETIMG/" + ui->comboBox_userFolder->currentText();

    if (!ui->comboBox_userFolder->currentText().isEmpty())
    {
        QDir dir;
        if(!dir.exists(path))
        {
            dir.mkdir(path);
        }

        for(int i=0; i<10 ; i++)
        {
            QString img = path + QString("/target_%1").arg(i);
            if(!QFile::exists(img + ".jpg"))
            {
                imageCapture_->capture(img);
                break;
            }
        }
    }
}

