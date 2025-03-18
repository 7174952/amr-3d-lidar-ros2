#ifndef SUBWINDOW_DEVICE_H
#define SUBWINDOW_DEVICE_H

#include <QDialog>

namespace Ui {
class SubWindow_Device;
}

class SubWindow_Device : public QDialog
{
    Q_OBJECT

public:
    explicit SubWindow_Device(QWidget *parent = nullptr);
    ~SubWindow_Device();

private:
    Ui::SubWindow_Device *ui;
};

#endif // SUBWINDOW_DEVICE_H
