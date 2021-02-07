#include "semicontrol_interface.h"
#include <QDebug>

Semicontrol_interface::Semicontrol_interface(QObject *parent) : QObject(parent)
{
   ptr_servo_ = new ServoDriver();
   openflag_ = ptr_servo_->Open();
   // ptr_servo_->Poweron();
}

void Semicontrol_interface::thrusterReceived(const QVector<double> &msg)
{


    std::vector<double> revo_speed, angle;
        for (int i = 0; i < 3; ++i)
        {
            revo_speed.push_back(10 * sqrt(msg.at(i)*coeff_force_ / coeff_thruster_));
            angle.push_back(msg.at(i + 3) / 2 / 180);
//            qDebug() << i << "revo_speed.at(0): " << revo_speed.at(i);
//            qDebug() << i << "angle.at(0): " << angle.at(i);
//            qDebug() << "-------11-------";
//          if(openflag_)
//          {
//            ptr_servo_->SetSpeed(thruster_id.at(i), revo_speed.at(i));
//            ptr_servo_->SetAngleAbs(thruster_id.at(i + 3), angle.at(i));
//          }
        }
        ptr_servo_->SetSpeed(thruster_id.at(0), revo_speed.at(0));
        ptr_servo_->SetSpeed(thruster_id.at(1), -revo_speed.at(1));
        ptr_servo_->SetSpeed(thruster_id.at(2), revo_speed.at(2));
        ptr_servo_->SetAngleAbs(thruster_id.at(3), angle.at(0));
        ptr_servo_->SetAngleAbs(thruster_id.at(4), angle.at(1));
        ptr_servo_->SetAngleAbs(thruster_id.at(5), angle.at(2));
        qDebug() << "-------22-------";
}

void Semicontrol_interface::servo_close()
{
    ptr_servo_->Poweroff();
    ptr_servo_->Close();
    delete ptr_servo_;
}
