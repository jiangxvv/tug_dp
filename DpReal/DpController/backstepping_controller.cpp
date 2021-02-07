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

#include "backstepping_controller.h"

#include <cmath>
#include <cstdio>

BackSteppingController::BackSteppingController(const VesselControlProperty& vessel, const double& step_size,
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

  for (size_t i = 0; i < 12; i++)
  {
    internal_state_.push_back(0);
  }
}

Eigen::Matrix3d BackSteppingController::RotationMatrixTranspose(const double& angle)
{
  Eigen::Matrix3d rz;
  rz << std::cos(angle), std::sin(angle), 0, -std::sin(angle), std::cos(angle), 0, 0, 0, 1;
  return rz;
}

Eigen::Matrix3d BackSteppingController::RotationMatrix(const double& angle)
{
  Eigen::Matrix3d rz;
  rz << std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle), 0, 0, 0, 1;
  return rz;
}

Eigen::Matrix3d BackSteppingController::RotationMatrixDerivative(const double& angle, const double& rate)
{
  Eigen::Matrix3d out;
  out << -std::sin(angle), -std::cos(angle), 0, std::cos(angle), -std::sin(angle), 0, 0, 0, 0;
  return rate * out;
}

void BackSteppingController::ComputeControl()
{
  // vessel state
  Eigen::Vector3d position, velocity, acceleration, velocity_setpoint;
  Eigen::Vector3d dot_eta_r, ddot_eta_r, dot_nu_r, nu_r;
  position << vessel_position_.at(0), vessel_position_.at(1), vessel_position_.at(2);
  velocity << vessel_velocity_.at(0), vessel_velocity_.at(1), vessel_velocity_.at(2);
  acceleration << vessel_acceleration_.at(0), vessel_acceleration_.at(1), vessel_acceleration_.at(2);

  // compute tracking error and s
  Eigen::Matrix3d rotation_matrix_T = RotationMatrixTranspose(position(2));
  Eigen::Matrix3d rotation_matrix = RotationMatrix(position(2));

  tracking_error_ << position_error_.at(0), position_error_.at(1), position_error_.at(2);
  velocity_setpoint << velocity_setpoint_.at(0), velocity_setpoint_.at(1), velocity_setpoint_.at(2);
  dot_eta_r = velocity_setpoint - Lambda_ * tracking_error_;
  s_ = rotation_matrix * velocity - dot_eta_r;
  nu_r = rotation_matrix_T * dot_eta_r;

  ddot_eta_r = -Lambda_ * (rotation_matrix * velocity - velocity_setpoint);
  dot_nu_r = rotation_matrix_T * ddot_eta_r -
             rotation_matrix_T * RotationMatrixDerivative(position(2), velocity(2)) * rotation_matrix_T * dot_eta_r;

  printf("--------------------------------------\n");
  printf("tracking error: %5.2f, %5.2f, %5.2f\n", position_error_.at(0), position_error_.at(1),
         position_error_.at(2) * 57.3);
  printf("velocity: %5.2f, %5.2f, %5.2f\n", velocity(0), velocity(1), velocity(2) * 57.3);
  printf("acceleration: %5.2f, %5.2f, %5.2f\n", acceleration(0), acceleration(1), acceleration(2) * 57.3);

  // compute control output
  Eigen::Vector3d control = M_ * dot_nu_r + D_ * nu_r - rotation_matrix_T * (Kp_ * tracking_error_ + Kd_ * s_);

  for (size_t i = 0; i < 3; i++)
  {
    internal_state_.at(i) = dot_nu_r(i);
    internal_state_.at(i + 3) = nu_r(i);
    internal_state_.at(i + 6) = tracking_error_(i);
    internal_state_.at(i + 9) = s_(i);
  }

  output_ = { control(0), control(1), control(2) };
  printf("control: %5.2f, %5.2f, %5.2f\n", output_.at(0), output_.at(1), output_.at(2));
}
