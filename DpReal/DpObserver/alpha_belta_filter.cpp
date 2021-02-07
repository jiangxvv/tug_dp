#include "alpha_belta_filter.h"
#include <cmath>
#include <functional>

AlphaBeltaFilter::AlphaBeltaFilter(const double& alpha,
                                   const double& belta,
                                   const double& time_step)
    : alpha_(alpha), belta_(belta), time_step_(time_step)
{

}


Eigen::Matrix3d AlphaBeltaFilter::rotate_matrix(const double &fai)
{
    Eigen::Matrix3d rm;
    rm<<std::cos(fai), -std::sin(fai), 0,
        std::sin(fai), std::cos(fai), 0,
            0, 0, 1;
    return rm;
}

Eigen::Matrix3d AlphaBeltaFilter::rotate_matrix_transpose(const double &fai)
{
    Eigen::Matrix3d rm;
    rm<<std::cos(fai), std::sin(fai), 0,
            -std::sin(fai), std::cos(fai), 0,
            0, 0, 1;
    return rm;

}


void AlphaBeltaFilter::computeFilter()
{
    //setQualysisInfo();
    pos_body_ = rotate_matrix(pos_[2]) * pos_;
    pos_body_hat_ = pos_init_ + time_step_ * vel_init_;
    vel_hat_ = vel_init_;

    Eigen::Vector3d r_hat;
    r_hat = pos_body_ - pos_body_hat_;

    pos_body_hat_ += alpha_ * r_hat;
    vel_hat_ += belta_ / time_step_ * r_hat;

    pos_init_ = pos_body_hat_;
    vel_init_ = vel_hat_;

    pos_hat_ = rotate_matrix_transpose(pos_body_hat_[2]) * pos_body_hat_;


}


