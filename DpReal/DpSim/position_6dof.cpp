#include "position_6dof.h"
#include <QDebug>

Position_6DOF::Position_6DOF(QObject *parent) : QObject(parent)
{
    qDebug() << "position 实例化 position 6dof";
    position_6DOF_interface = new Position_interface();
    qDebug() << "position 实例化 position 6dof 成功";
    position_6DOF_interface->moveToThread(&position_thread);

    connect(&position_thread, &QThread::finished, position_6DOF_interface,
            &QObject::deleteLater);
    connect(&position_thread, &QThread::started, position_6DOF_interface, Position_interface::StartPosition);
    connect(position_6DOF_interface, &Position_interface::positionReady, this,
            &Position_6DOF::positionReady);
    connect(this, &Position_6DOF::virtualpositionReceived, position_6DOF_interface,
            &Position_interface::virtualpositionReceived);


    position_thread.start();

}

Position_6DOF::~Position_6DOF()
{
    position_thread.quit();
    position_thread.wait();
    qDebug() << "position has exited";
}
