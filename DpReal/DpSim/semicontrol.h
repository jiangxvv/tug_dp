#ifndef SEMICONTROL_H
#define SEMICONTROL_H

#include <QObject>
#include <QString>
#include <QThread>
#include <QVector>
#include "controller.h"
#include "semicontrol_interface.h"

class Semicontrol : public QObject
{
    Q_OBJECT

      QThread semicontrol_thread;      

     Semicontrol_interface *semi_control_interface = nullptr;

public:
    explicit Semicontrol(QObject *parent = nullptr);
    ~Semicontrol();
signals:
    void thrusterReceived(const QVector<double> &msg);
    void servo_close();
};

#endif // SEMICONTROL_H
