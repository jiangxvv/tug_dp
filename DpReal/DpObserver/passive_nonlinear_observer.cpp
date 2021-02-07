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

#include "passive_nonlinear_observer.h"
#include <cmath>
#include <functional>

Eigen::Matrix3d PassiveNonlinearObserver::RotationMatrix(const double &angle) {
  Eigen::Matrix3d rz;
  rz << std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle),
      0, 0, 0, 1;
  return rz;
}

Eigen::Matrix3d
PassiveNonlinearObserver::RotationMatrixTranspose(const double &angle) {
  Eigen::Matrix3d rz;
  rz << std::cos(angle), std::sin(angle), 0, -std::sin(angle), std::cos(angle),
      0, 0, 0, 1;
  return rz;
}

PassiveNonlinearObserver::PassiveNonlinearObserver(
    const std::vector<double> &mass, const std::vector<double> &added_mass,
    const std::vector<double> &damping, const std::vector<double> &stiffness,
    const std::vector<double> &k3, const std::vector<double> &k4,
    const std::vector<double> &peak_frequency,
    const std::vector<double> &cutoff_frequency, const double &T,
    const double &step_size, const std::vector<double> &init_position)
    : omega_(peak_frequency), omega_c_(cutoff_frequency),
      step_size_(step_size) {
  // define the parameter matrices
  for (size_t i = 0; i < 3; i++) {
    M_(i, i) = mass.at(i) + added_mass.at(i);
    D_(i, i) = damping.at(i);
    G_(i, i) = stiffness.at(i);
    T_inv_(i, i) = 1 / T;
  }

  M_inv_ = M_.inverse();

  Aw_.topRightCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);
  Aw_.bottomLeftCorner(3, 3) =
      Eigen::Matrix3d(((Eigen::Vector3d() << -std::pow(omega_.at(0), 2),
                        -std::pow(omega_.at(1), 2), -std::pow(omega_.at(2), 2))
                           .finished())
                          .asDiagonal());
  Aw_.bottomRightCorner(3, 3) = Eigen::Matrix3d(
      ((Eigen::Vector3d() << -2 * lambda_ * omega_.at(0),
        -2 * lambda_ * omega_.at(1), -2 * lambda_ * omega_.at(2))
           .finished())
          .asDiagonal());
  Cw_.topRightCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);

  // define the gain matrices
  for (size_t i = 0; i < 3; i++) {
    K1_(i, i) = -2 * omega_c_.at(i) * (1 - lambda_) / omega_.at(i);
    K1_(i + 3, i) = 2 * omega_.at(i) * (1 - lambda_);
    K2_(i, i) = omega_c_.at(i);
    K3_(i, i) = k3.at(i);
    K4_(i, i) = k4.at(i);
  }

  // initial eta
  for (size_t i = 0; i < 3; i++) {
    eta_(i) = init_position.at(i);
  }
}

void PassiveNonlinearObserver::odeSystemFunction(
    const std::vector<double> &state, std::vector<double> &state_derivative,
    const double t) {
  xi_ << state[0], state[1], state[2], state[3], state[4], state[5];
  eta_ << state[6], state[7], state[8];
  b_ << state[9], state[10], state[11];
  nu_ << state[12], state[13], state[14];
  error_ << input_[0] - output_[0], input_[1] - output_[1],
      input_[2] - output_[2];

  Vector15d dy = StateDerivative(xi_, eta_, b_, nu_, error_, input_[2]);

  for (size_t index = 0; index < 15; ++index) {
    state_derivative[index] = dy[index];
  }
}

PassiveNonlinearObserver::Vector15d PassiveNonlinearObserver::StateDerivative(
    const PassiveNonlinearObserver::Vector6d &xi, const Eigen::Vector3d &eta,
    const Eigen::Vector3d &b, const Eigen::Vector3d &nu,
    const Eigen::Vector3d &error, const double &y3) const {
  Vector15d state_derivative;
  Vector6d dot_xi;
  Eigen::Vector3d dot_eta, dot_b, dot_nu;

  // compute the state derivatives
  dot_xi = Aw_ * xi + K1_ * error;
  dot_eta = RotationMatrix(y3) * nu + K2_ * error;
  dot_b = -T_inv_ * b + K3_ * error;
  dot_nu = M_inv_ * (-D_ * nu - G_ * eta + RotationMatrixTranspose(y3) * b +
                     actuation_ + RotationMatrixTranspose(y3) * K4_ * error);

  state_derivative << dot_xi, dot_eta, dot_b, dot_nu;
  return state_derivative;
}

void PassiveNonlinearObserver::OneStepIntegration(
    const std::vector<double> &measurement) {
  input_ = measurement;

  // assign values to state
  for (size_t i = 0; i < 6; i++) {
    state_.at(i) = xi_(i);
  }
  for (size_t i = 0; i < 3; i++) {
    state_.at(i + 6) = eta_(i);
  }
  for (size_t i = 0; i < 3; i++) {
    state_.at(i + 9) = b_(i);
  }
  for (size_t i = 0; i < 3; i++) {
    state_.at(i + 12) = nu_(i);
  }

  // perform a one-step integration
  stepper_.do_step(std::bind(&PassiveNonlinearObserver::odeSystemFunction, this,
                             std::placeholders::_1, std::placeholders::_2,
                             std::placeholders::_3),
                   state_, 0, step_size_);

  // save new values to variables
  xi_ << state_[0], state_[1], state_[2], state_[3], state_[4], state_[5];
  eta_ << state_[6], state_[7], state_[8];
  b_ << state_[9], state_[10], state_[11];
  nu_ << state_[12], state_[13], state_[14];

  updateOuput();
}

std::array<double, 3> PassiveNonlinearObserver::getAcceleration() {
  Eigen::Vector3d acc;
  acc = M_inv_ *
        (-D_ * nu_ - G_ * eta_ + RotationMatrixTranspose(input_[2]) * b_ +
         actuation_ + RotationMatrixTranspose(input_[2]) * K4_ * error_);
  std::array<double, 3> out = {{acc(0), acc(1), acc(2)}};
  return out;
}
