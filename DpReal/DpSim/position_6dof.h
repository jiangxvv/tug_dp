#ifndef POSITION_6DOF_H
#define POSITION_6DOF_H

#include <QObject>
#include <QString>
#include <array>
#include <QThread>
#include <position_interface.h>

class Position_6DOF : public QObject
{
    Q_OBJECT

    QThread position_thread;

    Position_interface *position_6DOF_interface;

public:
    explicit Position_6DOF(QObject *parent = nullptr);
    ~Position_6DOF();

signals:
    //void positionReady(const std::array<double, 6> &msg);
    //void virtualpositionReceived(const std::array<double, 6> &msg);
    void positionReady(const QVector<double> &msg);
    void virtualpositionReceived(const QVector<double> &msg);

};

#endif // POSITION_6DOF_H
