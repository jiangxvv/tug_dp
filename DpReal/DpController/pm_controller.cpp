/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bo Li <lither@sjtu.edu.cn>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bo Li nor the names of its contributors may
 *     be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <cmath>
#include <cstdio>
#include "pm_controller.h"

PmController::PmController(const VesselControlProperty& vessel, const double& step_size,
                           const std::array<double, 3>& Lambda, const std::array<double, 3>& Kp,
                           const std::array<double, 3>& Kd)
  : BasicController(vessel, step_size)
{
  M_(0, 0) = mass_ + a11_;
  M_(1, 1) = mass_ + a22_;
  M_(1, 2) = mass_ * xg_ + a23_;
  M_(2, 1) = M_(1, 2);
  M_(2, 2) = Izz_ + a33_;
  D_(0, 0) = d11_;
  D_(1, 1) = d22_;
  D_(2, 2) = d33_;

  for (size_t i = 0; i < 3; i++)
  {
    Lambda_(i, i) = Lambda.at(i);
    Kp_(i, i) = Kp.at(i);
    Kd_(i, i) = Kd.at(i);
  }
}

Eigen::Matrix3d PmController::RotationMatrixTranspose(const double& angle)
{
  Eigen::Matrix3d rz;
  rz << std::cos(angle), std::sin(angle), 0, -std::sin(angle), std::cos(angle), 0, 0, 0, 1;
  return rz;
}

Eigen::Matrix3d PmController::RotationMatrix(const double& angle)
{
  Eigen::Matrix3d rz;
  rz << std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle), 0, 0, 0, 1;
  return rz;
}

Eigen::Matrix3d PmController::RotationMatrixDerivative(const double& angle, const double& rate)
{
  Eigen::Matrix3d out;
  out << -std::sin(angle), -std::cos(angle), 0, std::cos(angle), -std::sin(angle), 0, 0, 0, 0;
  return rate * out;
}

Eigen::Matrix3d PmController::TMatrix() const
{
  Eigen::Matrix3d H_inverse = Eigen::Matrix3d::Identity();

  double rho = state_.at(0) < 1 ? 1 : state_.at(0);

  H_inverse(1, 1) = 1.0 / rho;
  return H_inverse * RotationMatrixTranspose(state_.at(1) - state_.at(2));
}

Eigen::Matrix3d PmController::TMatrixTranspose() const
{
  Eigen::Matrix3d out = TMatrix();
  return out.transpose();
}

Eigen::Matrix3d PmController::TMatrixInverse() const
{
  Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
  H(1, 1) = state_.at(0);
  return RotationMatrix(state_.at(1) - state_.at(2)) * H;
}

void PmController::ComputeControl()
{
  // vessel state
  Eigen::Vector3d x, dot_x;
  Eigen::Vector3d velocity;  // vessel veloctiy in the cartesian coordinate
  x << state_.at(0), state_.at(1), state_.at(2);
  velocity << vessel_velocity_.at(0), vessel_velocity_.at(1), vessel_velocity_.at(2);
  dot_x = TMatrix() * velocity;

  Eigen::Vector3d dot_x_r, ddot_x_r;
  tracking_error_ << x(0) - state_setpoint_.at(0), x(1) - state_setpoint_.at(1), x(2) - state_setpoint_.at(2);
  dot_x_r = -Lambda_ * tracking_error_;
  ddot_x_r = -Lambda_ * dot_x;

  Eigen::Matrix3d T_inv = TMatrixInverse();
  Eigen::Vector3d Z2 = dot_x + Lambda_ * tracking_error_;

  // compute control output
  Eigen::Vector3d control =
      M_ * T_inv * ddot_x_r + D_ * T_inv * dot_x_r - TMatrixTranspose() * (Kp_ * tracking_error_ + Kd_ * Z2);
  output_ = { control(0), control(1), control(2) };

//  printf("control: %5.2f, %5.2f, %5.2f\n", output_.at(0), output_.at(1), output_.at(2));
}

std::array<double, 3> PmController::ComputeState()
{
  state_.at(0) = std::sqrt(std::pow(vessel_position_.at(0), 2) + std::pow(vessel_position_.at(1), 2));
  state_.at(1) = std::atan2(vessel_position_.at(1), vessel_position_.at(0));
  state_.at(2) = vessel_position_.at(2);

//  printf("state: %5.2f, %5.2f, %5.2f\n", state_.at(0), state_.at(1), state_.at(2));

  return state_;
}
