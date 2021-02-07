#include "position_interface.h"
#include <array>
#include <iostream>
#include <memory>
#include <QVector>
#include <QDebug>
#include <windows.h>

Position_interface::Position_interface()
{
    //ptr_position_ = std::make_unique<Position>("192.168.1.40", 22222);
    ptr_position_ = std::make_unique<Position>();
    ptr_position_->GetPosition();

}

void Position_interface::RunPosition()
{
    bool runposition_ing = true;
    qDebug("\n---- Position Information Start ----");
    ptr_position_->GetPosition();
    std::array<double, 6> zero_position_ = ptr_position_->position_6DOF_;
    ptr_position_->SetZeroPosition(zero_position_);
    while(runposition_ing)
    {
        PublishPosition();
    }
    qDebug("\n---- Position Information Over ----");
}

void Position_interface::StartPosition()
{
    qDebug("---- Position Start ----");
    RunPosition();
    qDebug("---- Position Over ----");
}

void Position_interface::PublishPosition()
{
    for(int i = 0; i < 19; i ++)
    {
       ptr_position_->GetPosition();
       Sleep(50);
    }
    ptr_position_->GetPosition();
    qDebug() << "ptr_position_->GetPosition() ";
//    Sleep(1000);
    std::array<double, 6> position_transformed_ = ptr_position_->PositionOutput();
    qDebug() << "ptr_position_->PositionOutput()";
    QVector<double> totalpositionMessage = {position_transformed_.at(0),
                                            position_transformed_.at(1),
                                            position_transformed_.at(2),
                                            position_transformed_.at(3),
                                            position_transformed_.at(4),
                                            position_transformed_.at(5),
                                           };
    // 速度滤波类
//    QVector<double> totalpositionMessage = {0.0,
//                                            0.0,
//                                            0.0,
//                                            0.0,
//                                            0.0,
//                                            0.0,
//                                           };

    emit positionReady(totalpositionMessage);
//    Sleep(1000);
    Sleep(50);
    qDebug() << "position_transformed_.at(0): " << position_transformed_.at(0);
}



void Position_interface::virtualpositionReceived(const QVector<double> &msg)
{
//    std::array<double, 6> virtualposition_6DOF;
//    std::array<double, 6> totalpostion_6DOF;

//    for(int i = 0; i < 6; i++)
//    {
//        virtualposition_6DOF.at(i) = msg.at(i);
//    }

//    ptr_position_->SetVirtualPosition(virtualposition_6DOF);
//    totalpostion_6DOF = ptr_position_->PositionOutput();

//    QVector<double> totalpositionMessage = { totalpostion_6DOF.at(0),
//                                             totalpostion_6DOF.at(1),
//                                             totalpostion_6DOF.at(2),
//                                             totalpostion_6DOF.at(3),
//                                             totalpostion_6DOF.at(4),
//                                             totalpostion_6DOF.at(5)
//                                           };
//    QVector<double> totalpositionMessage = { 50.0,
//                                             60.0,
//                                             0.0,
//                                             0.0,
//                                             0.0,
//                                             0.0
//                                           };

//    emit positionReady(totalpositionMessage);
//    qDebug() << totalpositionMessage.at(1);

}
