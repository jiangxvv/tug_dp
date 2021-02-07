#ifndef SEMICONTROL_INTERFACE_H
#define SEMICONTROL_INTERFACE_H

#include <QObject>
#include "servodriver.h"
#include <memory.h>

class Semicontrol_interface : public QObject
{
    Q_OBJECT
public:
    explicit Semicontrol_interface(QObject *parent = nullptr);
    ServoDriver *ptr_servo_;
    const double Kt_ = 0.6;
    const double rho_ = 1025.0;
    const double D_ = 0.076;
    const double coeff_force_=1000 / std::pow(50, 3);
    const double coeff_thruster_=Kt_*rho_*std::pow(D_, 4);
    bool openflag_ = false;
    std::array<int, 6> thruster_id = { { 0, 1, 3, 5, 4, 2 } };
//    std::vector<double> rev_speed_;
//    std::vector<double> angle_;

public slots:
    void thrusterReceived(const QVector<double> &msg);
    void servo_close();
};

#endif // SEMICONTROL_INTERFACE_H
