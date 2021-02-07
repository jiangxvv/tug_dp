#ifndef ALPHA_BELTA_FILTER_H
#define ALPHA_BELTA_FILTER_H

#include "dpobserver_global.h"
#include <Eigen>
#include <array>
#include <vector>



class DPOBSERVERSHARED_EXPORT AlphaBeltaFilter
//class  AlphaBeltaFilter
{

private:
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    double alpha_;
    double belta_;
    double time_step_;

    Eigen::Vector3d pos_, vel_;

    Eigen::Vector3d pos_init_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_init_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d pos_body_hat_;
    Eigen::Vector3d pos_body_;
    Eigen::Vector3d pos_hat_, vel_hat_;




public:

EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AlphaBeltaFilter(const double& alpha,
                     const double& belta,
                     const double& time_step);

    Eigen::Matrix3d rotate_matrix(const double& fai);

    Eigen::Matrix3d rotate_matrix_transpose(const double& fai);

    void computeFilter();




    ~AlphaBeltaFilter() {}

    Eigen::Vector3d getPosition()
    {
        return pos_hat_;
    }

    Eigen::Vector3d getVelocity()
    {
        return vel_hat_;
    }

    void setQualysisInfo(const Vector6d& pos)
    {
        pos_<<pos[0], pos[1], pos[2];

    }
};

#endif // ALPHA_BELTA_FILTER_H
