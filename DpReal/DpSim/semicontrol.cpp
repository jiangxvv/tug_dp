#include "semicontrol.h"
#include <QDebug>

Semicontrol::Semicontrol(QObject *parent) : QObject(parent)
{
    semi_control_interface = new Semicontrol_interface();
    semi_control_interface->moveToThread(&semicontrol_thread);

    connect(&semicontrol_thread, &QThread::finished, semi_control_interface,
            &QObject::deleteLater);
    connect(this, &Semicontrol::thrusterReceived,semi_control_interface,
            &Semicontrol_interface::thrusterReceived);
    connect(this, &Semicontrol::servo_close, semi_control_interface,
            &Semicontrol_interface::servo_close);
    semicontrol_thread.start();
}

Semicontrol::~Semicontrol()
{
    semicontrol_thread.quit();
    semicontrol_thread.wait();
    qDebug() << "semicontrol has exited";

}


