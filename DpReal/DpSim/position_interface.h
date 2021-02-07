#ifndef POSITION_INTERFACE_H
#define POSITION_INTERFACE_H

#include <QObject>
#include <iostream>
#include <memory>
#include <position.h>
#include <QVector>

class Position_interface : public QObject
{
    Q_OBJECT

private:
    std::unique_ptr<Position> ptr_position_;

public:
    Position_interface();
    ~Position_interface() override = default;

signals:
    void positionReady(const QVector<double> &msg);
    //void positionReady(const std::array<double, 6> &msg);

public slots:
    void virtualpositionReceived(const QVector<double> &msg);
    void StartPosition();
    void RunPosition();
    void PublishPosition();


};

#endif // POSITION_INTERFACE_H
