#include "servodriver.h"
#include<iostream>
#include<cmath>
#include"triopclib.h"

ServoDriver::ServoDriver(QObject *parent): QObject(parent)
{
    TrioPC = new TrioPCLib::TrioPC();
//    Open();
//    Poweron();
   std::cout<<"new a servo"<<std::endl;

//    TrioPC = new TrioPCLib::_DTrioPC(this);
//    TrioPC = new TrioPCLib::TrioPC(this);
}


ServoDriver::~ServoDriver()
{
    Poweroff();
    TrioPC->Close(0);
    printf("test");
    delete TrioPC;
}



bool ServoDriver::Open()
{
    TrioPC->SetHost("192.168.0.251");
    bool openFlag = TrioPC->Open(2, 0);
    if (openFlag == false)
        std::cout<<"Open failed!"<<std::endl;
    else
        std::cout<<"Connection has been established"<<std::endl;
    TrioPC->Execute("WDOG=ON");
    return openFlag;

}

bool ServoDriver::Close()
{

    TrioPC->Close();
    return true;
}

void ServoDriver::Poweron()
{
    TrioPC->Execute("WDOG=ON");
//    for(int i=0; i<6; i++)
//        SinglePoweron(i);

}

void ServoDriver::Poweroff()
{
    TrioPC->Execute("WDOG=OFF");
//    for(int i=0; i<6; i++)
//        SinglePoweroff(i);
}

void ServoDriver::SinglePoweron(const int& axis_id)
{
    TrioPC->Base(1, axis_id);
    TrioPC->SetVariable("WDOG", 1.0);
}

void ServoDriver::SinglePoweroff(const int& axis_id)
{
    TrioPC->Base(1, axis_id);
    TrioPC->SetVariable("WDOG", 0.0);
}



void ServoDriver::SetAngle(const int &axis_id, const double &axis_angle)
{

     TrioPC->Cancel(1, QVariant(axis_id));
    if(axis_angle>=0)
    {
//        TrioPC->MoveRel(1, QVariant(axis_angle), axis_id);
        TrioPC->MoveRel(1, QVariant(axis_angle), QVariant(axis_id));
//        TrioPC->Forward(QVariant(axis_id));
    }
    else
    {
        TrioPC->MoveRel(1, QVariant(axis_angle), QVariant(axis_id));
//        TrioPC->Reverse(QVariant(axis_id));

    }

}

void ServoDriver::SetAngleAbs(const int &axis_id, const double &axis_angle_abs)
{
  //  TrioPC->Base(1, axis_id);
    TrioPC->Cancel(1, QVariant(axis_id));
    TrioPC->MoveAbs(1, QVariant(axis_angle_abs), QVariant(axis_id));
}


void ServoDriver::SetSpeed(const int& axis_id, const double& axis_speed)
{
 //   double time_limit=0.005;
    //double speed_rel;
  //  double accel;
   // TrioPC->Base(1, axis_id);
    TrioPC->Cancel(1, QVariant(axis_id));
   // speed_rel = GetSpeed(axis_id);
    if(axis_speed>=0)
    {
        TrioPC->SetAxisVariable("SPEED", axis_id, axis_speed);
        TrioPC->Forward(QVariant(axis_id));
//        if(axis_speed>=speed_rel)
//        {
//            accel=(axis_speed-speed_rel)/time_limit;
//            TrioPC->SetAxisVariable("ACCEL", axis_id, accel);
//        }
//        else {
//            accel=(speed_rel-axis_speed)/time_limit;
//            TrioPC->SetAxisVariable("DECEL", axis_id, accel);
//        }



    }
    else
    {
        TrioPC->SetAxisVariable("SPEED", axis_id, axis_speed);
        TrioPC->Reverse(QVariant(axis_id));
//        accel = std::abs(axis_speed-speed_rel)/time_limit;
//        if(axis_speed>=speed_rel)
//            TrioPC->SetAxisVariable("DECEL", axis_id, accel);
//        else
//            TrioPC->SetAxisVariable("ACCEL", axis_id, accel);

    }

}



double ServoDriver::GetAngle(const int& axis_id)
{   double angle;
    TrioPC->GetAxisVariable("DPOS", axis_id, angle);
    return angle;
}

double ServoDriver::GetSpeed(const int &axis_id)
{
    double speed;
    TrioPC->GetAxisVariable("MSPEED", axis_id, speed);
    return speed;
}


