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

#include "controller_interface.h"
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <QDebug>

//#include "pm_setpoint_interface.h"

ControllerInterface::ControllerInterface(
    const VesselControlProperty &vessel,
    const ThrustAllocationConfiguration &config,
    const std::array<double, 3> &PositionSetPoint, const double &step_size,
    std::string save_path)
    : step_size_(step_size), output_directory_(std::move(save_path)) {

    qDebug() << "--- 进入 controller interface 构造函数 ---";


    L << 0.415, 0.3,
         0.415, -0.3,
         -0.545, 0.0;
      std::array<double, 3> Kp = {{0.3825, 0.51, 0.4131}};
      std::array<double, 3> Kd = {{10.37, 4.08, 3.3048}};

  ptr_controller_ = std::make_unique<PIDController>(vessel, 0.1, Kp, Kd);

//  ptr_controller_ = std::make_unique<FuzzyController>(vessel, 0.1, 10);
  ptr_controller_->setPositionSetpoint({0.0, 0.0, 0.0});

  // define the thrust allocator
  ptr_allocator_ = std::make_unique<inverse_thrust_allocation>(L);

  step_interval_ = (unsigned int)ptr_controller_->getStepSize() / step_size_ + 1;

  for (int i = 0; i < 3 ; i++)
  {
      thruster_force_.push_back(0);
      thruster_angle_.push_back(0);
  }

  qDebug() << "--- controller interface 构造函数 ---";

}

void ControllerInterface::messageReceived(const QVector<double> &msg) {

//  qDebug() << "--- controller interface messageReceived ---";
  std::array<double, 3> position, velocity, acceleration;

  position.at(0) = msg.at(1); // first element is timestamp
  position.at(1) = msg.at(2);
  position.at(2) = msg.at(3);

  velocity.at(0) = msg.at(4);
  velocity.at(1) = msg.at(5);
  velocity.at(2) = msg.at(6);

  acceleration.at(0) = msg.at(7);
  acceleration.at(1) = msg.at(8);
  acceleration.at(2) = msg.at(9);

  ptr_controller_->setVesselPosition(position);
  ptr_controller_->setVesselVelocity(velocity);
  ptr_controller_->setVesselAcceleration(acceleration);

  ptr_controller_->ComputeControl();
//  ptr_controller_->FuzzyControl();
  std::array<double, 3> output = ptr_controller_->getOutput();

  for (int i = 0; i < 3; i++)
  {
      output_for_allocator[i] = output.at(i);
  }
  ptr_allocator_->compute(output_for_allocator);

//  for (size_t i = 0; i < 3; i++) {
//    control_output_.at(i) = output.at(i) / 1E3; // unit: kN
//  }

  for (int i = 0; i < 3 ; i++)
  {
      thruster_force_.at(i) = ptr_allocator_->f_[i];
      thruster_angle_.at(i) = ptr_allocator_->a_[i];
  }

  QVector<double> thrusterForceMessage = { thruster_force_.at(0),
                                           thruster_force_.at(1),
                                           thruster_force_.at(2),
                                           thruster_angle_.at(0),
                                           thruster_angle_.at(1),
                                           thruster_angle_.at(2)};
//  QVector<double> thrusterForceMessage = { 0.5,
//                                           0.5,
//                                           0.5,
//                                           0.0,
//                                           0.0,
//                                           0.0};
   emit thrusterReady(thrusterForceMessage);


  // save thrusters data
  if (qp_counter_ == 0) {
    ExportData(output_directory_);
  } else {
    ExportData(output_directory_, "app");
  }

  ++qp_counter_;

  static unsigned int counter = 0;
  ++counter;

//  for (int i = 0; i < 3; i++)
//  {
//      qDebug() << i << " : tau : " << output.at(i) << "   tau real : " << ptr_allocator_->tau_real_[i];
//  }

//  qDebug() << "-------------------------------";


  if (counter == step_interval_) {
    // publish control output
    QVector<double> control = {allocation_output_.at(0) * 1E3,
                               allocation_output_.at(1) * 1E3,
                               allocation_output_.at(2) * 1E3};
//      QVector<double> control = {5.0,
//                                 5.0,
//                                 5.0};
    emit messageReady(control);

    counter = 0;
  }

}

void ControllerInterface::setpointReceived(const QVector<double> &msg)
{
    std::array<double, 3> setpoint;
    setpoint.at(0) = msg.at(0);
    setpoint.at(1) = msg.at(1);
    setpoint.at(2) = msg.at(2);

    ptr_controller_->setPositionSetpoint(setpoint);
}

void ControllerInterface::ExportData(const std::string &save_directory,
                                     const std::string &mode) const {
  const std::string &control_file =
      save_directory + "control_allocation_file.txt";
  const std::string &force_file = save_directory + "thruster_force_file.txt";
  const std::string &angle_file = save_directory + "thruster_angle_file.txt";
  const std::string &controller_file = save_directory + "controller_file.txt";

  std::ofstream control_out, force_out, angle_out;
  std::ofstream controller_out;

  // open the files
  if (mode == "trunc") {
    control_out.open(control_file.c_str());
    force_out.open(force_file.c_str());
    angle_out.open(angle_file.c_str());
    controller_out.open(controller_file.c_str());
  } else {
    control_out.open(control_file.c_str(), std::ofstream::app);
    force_out.open(force_file.c_str(), std::ofstream::app);
    angle_out.open(angle_file.c_str(), std::ofstream::app);
    controller_out.open(controller_file.c_str(), std::ofstream::app);
  }

  // write data to the file
  control_out << std::fixed << std::setprecision(2) << qp_counter_ << '\t';
  control_out << std::setprecision(5);

  for (size_t i = 0; i < 3; i++) {
    control_out << std::scientific << control_output_.at(i) << '\t';
  }
  for (size_t i = 0; i < 3; i++) {
    control_out << std::scientific << allocation_output_.at(i) << '\t';
  }

  control_out << std::endl;

  // write thruster force data to the file
  force_out << std::fixed << std::setprecision(2) << qp_counter_ << '\t';
  force_out << std::setprecision(5);

  for (size_t i = 0; i < thruster_force_.size(); i++) {
    force_out << std::scientific << thruster_force_.at(i) << '\t';
  }

  force_out << std::endl;

  // write thruster angle data to the file
  angle_out << std::fixed << std::setprecision(2) << qp_counter_ << '\t';
  angle_out << std::setprecision(5);

  for (size_t i = 0; i < thruster_angle_.size(); i++) {
    angle_out << std::scientific << thruster_angle_.at(i) << '\t';
  }

  angle_out << std::endl;

  // backstepping data out
  controller_out << std::fixed << std::setprecision(2)
                 << qp_counter_ * step_size_ << '\t';
  controller_out << std::setprecision(5);

  for (size_t i = 0; i < controller_state_.size(); i++) {
    controller_out << std::scientific << controller_state_.at(i) << '\t';
  }

  controller_out << std::endl;

  // close the files
  control_out.close();
  force_out.close();
  angle_out.close();
  controller_out.close();
}
