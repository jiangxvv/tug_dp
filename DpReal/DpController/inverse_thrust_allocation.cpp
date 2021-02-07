#include "inverse_thrust_allocation.h"
#include<Eigen>
#include<iostream>
//#include<Math>
inverse_thrust_allocation::inverse_thrust_allocation(const Matrix32& L)
    : L_(L)
{

    B_<< 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1,
            -L_(0, 1), L_(0, 0), -L_(1, 1), L_(1, 0), -L_(2, 1), L_(2, 0);

//    std::cout<<B_<<std::endl;

}



void inverse_thrust_allocation::compute(const Eigen::Vector3d& tau)
{
    Matrix63 B_inv;
    B_inv = B_.transpose() * (B_ * B_.transpose()).inverse();
    Vector6d x;
    x = B_inv * tau;

    double fx1, fy1, fx2, fy2, fx3, fy3;
    fx1 = x[0]; fy1 = x[1];
    fx2 = x[2]; fy2 = x[3];
    fx3 = x[4]; fy3 = x[5];

    double f1, f2, f3, a1, a2, a3;   //degree

    f1 = std::sqrt(std::pow(fx1, 2) + std::pow(fy1, 2));
    f2 = std::sqrt(std::pow(fx2, 2) + std::pow(fy2, 2));
    f3 = std::sqrt(std::pow(fx3, 2) + std::pow(fy3, 2));

    a1 = std::atan2(fy1, fx1) * 180 / 3.14;
    a2 = std::atan2(fy1, fx1) * 180 / 3.14;
    a3 = std::atan2(fy1, fx1) * 180 / 3.14;

    f_<< f1, f2, f3;
    a_<< a1, a2, a3;


    tau_real_ = B_ * x;
//    std::cout<<"真实分配的合力: "<<tau_real_.transpose()<<std::endl;

}
