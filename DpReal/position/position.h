#ifndef POSITION_H
#define POSITION_H

#include <array>
#include <string>
#include "position_global.h"
#include "RTPacket.h"
#include "RTProtocol.h"


class POSITIONSHARED_EXPORT Position
{
protected:
    //6DOF data
    std::array<double, 6> vessel_position_ = { { 0, 0, 0, 0, 0, 0 } };
    std::array<double, 6> vessel_velocity_ = { { 0, 0, 0, 0, 0, 0 } };

private:
//    //6DOF实时数据
//    std::array<double, 6> position_6DOF_ = { {0, 0, 0, 0, 0, 0 } };
    //采零存储用
    std::array<double, 6> position_zero_ = { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    //转化到坐标轴上的位置，以采零处为原点
    std::array<double, 6> position_output_ = { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    //虚拟位移
    std::array<double, 6> virtual_postion_ = { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

public:
    //6DOF实时数据
    std::array<double, 6> position_6DOF_ = { {0, 0, 0, 0, 0, 0 } };
    CRTProtocol rtProtocol;
    Position();
    ~Position();
    void SetXg(const double &setxg);
    void SetYg(const double &setyg);
    void SetPsi(const double &setpsi);

    bool InitialPosition();
    //读取6DOF数据
    bool GetPosition();
    //
    std::array<double, 6> get_position_6dof_value() {return position_6DOF_;}
    //
    void GetZeroPosition(double &xg, double &yg, double &psi);
    //
    void SetZeroPosition(std::array<double, 6> &position_6DOF) {position_zero_ = position_6DOF;}
    //
    std::array<double, 6> PositionOutput();
    //虚拟位移接口
    void SetVirtualPosition(const std::array<double, 6> &virtual_position)
    {
        virtual_postion_ = virtual_position;
    }

};

#endif // POSITION_H
