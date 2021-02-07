#ifndef INVERSE_THRUST_ALLOCATION_H
#define INVERSE_THRUST_ALLOCATION_H

#include<Eigen>
#include "dpcontroller_global.h"

typedef Eigen::Matrix<double, 3, 2> Matrix32;
typedef Eigen::Matrix<double, 3, 6> Matrix36;
typedef Eigen::Matrix<double, 6, 3> Matrix63;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class DPCONTROLLERSHARED_EXPORT inverse_thrust_allocation
{
public:
    inverse_thrust_allocation( const Matrix32& L );
    Eigen::Vector3d tau_real_, tau_;

    Eigen::Vector3d f_, a_;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // method
    void compute(const Eigen::Vector3d& tau);

private:
//    typedef Eigen::Matrix<double, 3, 2> Matrix32;
//    typedef Eigen::Matrix<double, 3, 6> Matrix36;
//    typedef Eigen::Matrix<double, 6, 3> Matrix63;
//    typedef Eigen::Matrix<double, 6, 1> Vector6d;


    int N_;   // the number of the amuzith thrusters
    Matrix32 L_; // the position of the thrusters
    Matrix36 B_;








};

#endif // INVERSE_THRUST_ALLOCATION_H
