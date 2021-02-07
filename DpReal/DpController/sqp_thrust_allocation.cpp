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

#include "sqp_thrust_allocation.h"
#include <cmath>
#include <iostream>

SQPThrustAllocation::SQPThrustAllocation(
    const ThrustAllocationConfiguration &config, const double &time_step)
    : BasicThrustAllocation(config), time_step_(time_step) {
  // initialize the data for QP
  H_ = new double[(2 * total_thruster_number_ + 3) *
                  (2 * total_thruster_number_ + 3)]();
  A_ = new double[3 * (2 * total_thruster_number_ + 3)]();
  g_ = new double[2 * total_thruster_number_ + 3]();
  lb_ = new double[2 * total_thruster_number_ + 3]();
  ub_ = new double[2 * total_thruster_number_ + 3]();
  lbA_ = new double[3]();
  ubA_ = new double[3]();

  // initializae the solution
  xOpt_ = new double[2 * total_thruster_number_ + 3]();

  // define the Hessian matrix
  setH(config.weights);

  // setting up SQProblem object
  ptr_qproblem_ = std::unique_ptr<SQProblem>(
      new SQProblem(2 * total_thruster_number_ + 3, 3));

  // change printLevel
  Options myOptions;
  myOptions.printLevel = PL_NONE;
  ptr_qproblem_->setOptions(myOptions);

  // initialize thrust and azimuth angle
  setThrusterAngle(config.initial_azimuth);
  setThrusterForce(config.initial_thrust);
}

std::array<std::vector<double>, 3> SQPThrustAllocation::ThrustConfigMatrix(
    const std::vector<double> &azimuth_angle,
    const std::vector<double> &x_coordinate,
    const std::vector<double> &y_coordinate) const {
  std::array<std::vector<double>, 3> bMatrix;
  for (size_t idx = 0; idx < total_thruster_number_; ++idx) {
    bMatrix.at(0).push_back(std::cos(azimuth_angle.at(idx)));
    bMatrix.at(1).push_back(std::sin(azimuth_angle.at(idx)));
    bMatrix.at(2).push_back(
        -y_coordinate.at(idx) * std::cos(azimuth_angle.at(idx)) +
        x_coordinate.at(idx) * std::sin(azimuth_angle.at(idx)));
  }
  return bMatrix;
}

std::array<std::vector<double>, 3>
SQPThrustAllocation::ThrustConfigMatrixDerivative(
    const std::vector<double> &azimuth_angle,
    const std::vector<double> &x_coordinate,
    const std::vector<double> &y_coordinate) const {
  std::array<std::vector<double>, 3> bMatrix;
  for (size_t idx = 0; idx < total_thruster_number_; ++idx) {
    bMatrix.at(0).push_back(-std::sin(azimuth_angle.at(idx)));
    bMatrix.at(1).push_back(std::cos(azimuth_angle.at(idx)));
    bMatrix.at(2).push_back(
        y_coordinate.at(idx) * std::sin(azimuth_angle.at(idx)) +
        x_coordinate.at(idx) * std::cos(azimuth_angle.at(idx)));
  }
  return bMatrix;
}

int SQPThrustAllocation::initQProblem(const std::array<double, 3> &tau) {
  setA(thruster_force_, thruster_angle_);
  setG();
  setLB(thruster_force_, time_step_);
  setUB(thruster_force_, time_step_);
  setLBA(tau);
  setUBA(tau);

  int_t nWSR = 100;

  returnValue flag =
      ptr_qproblem_->init(H_, g_, A_, lb_, ub_, lbA_, ubA_, nWSR);

  if (flag == SUCCESSFUL_RETURN) {
    ptr_qproblem_->getPrimalSolution(xOpt_);

    for (size_t i = 0; i < total_thruster_number_; ++i) {
      thruster_force_.at(i) = xOpt_[i];
      thruster_angle_.at(i) += xOpt_[i + total_thruster_number_];
      // wrap thruster angle into [-pi, pi]
      thruster_angle_.at(i) = std::atan2(std::sin(thruster_angle_.at(i)),
                                         std::cos(thruster_angle_.at(i)));
    }
    for (size_t i = 0; i < 3; ++i) {
      error_.at(i) = xOpt_[2 * total_thruster_number_ + i];
    }

    // compute control output
    computeControlOutput(thruster_force_, thruster_angle_);

    return 0;
  } else {
    std::cerr << "The optimization problem cannot be solved!" << std::endl;
    return -1;
  }
}

int SQPThrustAllocation::solveQProblem(const std::array<double, 3> &tau) {
  setA(thruster_force_, thruster_angle_);
  setG();
  setLB(thruster_force_, time_step_);
  setUB(thruster_force_, time_step_);
  setLBA(tau);
  setUBA(tau);

  int_t nWSR = 100;

  returnValue flag =
      ptr_qproblem_->hotstart(H_, g_, A_, lb_, ub_, lbA_, ubA_, nWSR, 0);

  if (flag == SUCCESSFUL_RETURN) {
    ptr_qproblem_->getPrimalSolution(xOpt_);

    for (size_t i = 0; i < total_thruster_number_; ++i) {
      thruster_force_.at(i) = xOpt_[i];
      thruster_angle_.at(i) += xOpt_[i + total_thruster_number_];
      // wrap thruster angle into [-pi, pi]
      thruster_angle_.at(i) = std::atan2(std::sin(thruster_angle_.at(i)),
                                         std::cos(thruster_angle_.at(i)));
    }
    for (size_t i = 0; i < 3; ++i) {
      error_.at(i) = xOpt_[2 * total_thruster_number_ + i];
    }

    // compute control output
    computeControlOutput(thruster_force_, thruster_angle_);

    return 0;
  } else {
    std::cerr << "The optimization problem cannot be solved!" << std::endl;
    return -1;
  }
}

void SQPThrustAllocation::computeControlOutput(
    const std::vector<double> &thruster_force,
    const std::vector<double> &thruster_angle) {
  std::array<std::vector<double>, 3> BMatrix =
      ThrustConfigMatrix(thruster_angle, x_coordinates_, y_coordinates_);

  for (size_t i = 0; i < 3; i++) {
    double output = 0;
    for (size_t j = 0; j < total_thruster_number_; j++) {
      output += BMatrix.at(i).at(j) * thruster_force.at(j);
    }
    control_output_.at(i) = output;
  }
}

void SQPThrustAllocation::setH(const std::vector<double> &weights) {
  for (size_t i = 0; i < total_thruster_number_; i++) {
    H_[(2 * total_thruster_number_ + 3) * i + i] = weights.at(0);
    H_[(2 * total_thruster_number_ + 3) * total_thruster_number_ +
       (2 * total_thruster_number_ + 3) * i + total_thruster_number_ + i] =
        weights.at(1);
  }
  for (size_t i = 0; i < 3; i++) {
    H_[(2 * total_thruster_number_ + 3) * 2 * total_thruster_number_ +
       (2 * total_thruster_number_ + 3) * i + 2 * total_thruster_number_ + i] =
        weights.at(2);
  }
}

void SQPThrustAllocation::setA(const std::vector<double> &thruster_force,
                               const std::vector<double> &thruster_angle) {
  std::array<std::vector<double>, 3> BMatrix =
      ThrustConfigMatrix(thruster_angle, x_coordinates_, y_coordinates_);
  std::array<std::vector<double>, 3> BMatrixD = ThrustConfigMatrixDerivative(
      thruster_angle, x_coordinates_, y_coordinates_);

  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < total_thruster_number_; ++j) {
      A_[(2 * total_thruster_number_ + 3) * i + j] = BMatrix.at(i).at(j);
      A_[(2 * total_thruster_number_ + 3) * i + total_thruster_number_ + j] =
          BMatrixD.at(i).at(j) * thruster_force.at(j);
    }
  }
  A_[2 * total_thruster_number_] = 1;
  A_[4 * total_thruster_number_ + 4] = 1;
  A_[6 * total_thruster_number_ + 8] = 1;
}

void SQPThrustAllocation::setG() {
  for (size_t i = 0; i < total_thruster_number_; ++i) {
    g_[i] = 0;
  }
}

void SQPThrustAllocation::setLB(const std::vector<double> &thruster_force,
                                const double &time_step) {
  for (size_t i = 0; i < total_thruster_number_; ++i) {
    double Tcan1 = thruster_force.at(i) - thrust_rate_limit_.at(i) * time_step;
    double Tcan2 = thrust_lower_limit_.at(0);
    lb_[i] = Tcan1 > Tcan2 ? Tcan1 : Tcan2;
    lb_[i + total_thruster_number_] =
        -rotation_rate_limit_.at(i) * M_PI / 180 * time_step;
  }

  lb_[2 * total_thruster_number_] = -1E5;
  lb_[2 * total_thruster_number_ + 1] = -1E5;
  lb_[2 * total_thruster_number_ + 2] = -1E5;
}

void SQPThrustAllocation::setUB(const std::vector<double> &thruster_force,
                                const double &time_step) {
  for (size_t i = 0; i < total_thruster_number_; ++i) {
    double Tcan1 = thruster_force.at(i) + thrust_rate_limit_.at(i) * time_step;
    double Tcan2 = thrust_upper_limit_.at(0);
    ub_[i] = Tcan1 > Tcan2 ? Tcan2 : Tcan1;
    ub_[i + total_thruster_number_] =
        rotation_rate_limit_.at(i) * M_PI / 180 * time_step;
  }

  ub_[2 * total_thruster_number_] = 1E5;
  ub_[2 * total_thruster_number_ + 1] = 1E5;
  ub_[2 * total_thruster_number_ + 2] = 1E5;
}

void SQPThrustAllocation::setLBA(const std::array<double, 3> &tau) {
  for (size_t i = 0; i < 3; ++i) {
    lbA_[i] = tau.at(i) - 0.001;
  }
}

void SQPThrustAllocation::setUBA(const std::array<double, 3> &tau) {
  for (size_t i = 0; i < 3; ++i) {
    ubA_[i] = tau.at(i) + 0.001;
  }
}
