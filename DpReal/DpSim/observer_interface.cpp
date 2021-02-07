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

#include "observer_interface.h"
#include <fstream>
#include <QDebug>

ObserverInterface::ObserverInterface(const VesselObserverProperty &vessel,
                                     const double &step_size,
                                     const std::vector<double> &init_position,
                                     std::string save_path)
    : output_directory_(std::move(save_path)) {
  // define a passive nonlinear observer
  std::vector<double> k3 = {1E5, 1E5, 1E8};
  std::vector<double> k4 = {1E6, 1E6, 1E9};

  double alpha = 1.0;
  double belta = 1.0;

  ptr_observer_ =
      std::unique_ptr<PassiveNonlinearObserver>(new PassiveNonlinearObserver(
          vessel.mass, vessel.added_mass, vessel.damping, vessel.stiffness, k3,
          k4, {0.6, 0.6, 1.0}, {0.7, 0.7, 1.2}, 1000, step_size,
          init_position));
  ptr_alpha_belta_filter_ = std::unique_ptr<AlphaBeltaFilter>(new AlphaBeltaFilter(alpha, belta, step_size));
}

void ObserverInterface::motionMessageReceived(const QVector<double> &msg) {
  // set current time
  current_time_ = 0;
  ++step_counter_;
  QualysisInfo<<msg.at(0), msg.at(1), 0, 0, 0, msg.at(2);
  ptr_alpha_belta_filter_->setQualysisInfo(QualysisInfo);

  ptr_alpha_belta_filter_->computeFilter();

  std::array<double, 3> observer_vel;
  Eigen::Vector3d vel_hat;
  vel_hat = ptr_alpha_belta_filter_->getVelocity();

  observer_vel.at(0) = vel_hat(0);
  observer_vel.at(1) = vel_hat(1);
  observer_vel.at(2) = vel_hat(2);



  // history
  measurement_history_.at(0) = measurement_.at(0);
  measurement_history_.at(1) = measurement_.at(1);
  measurement_history_.at(2) = measurement_.at(2);
  // measured states
  measurement_.at(0) = msg.at(0);
  measurement_.at(1) = msg.at(1);
  measurement_.at(2) = msg.at(5);

//  std::array<double, 3> observer_vel;
//  for (int i = 0; i < 3; i++)
//  {
//      observer_vel.at(i) = (measurement_.at(i) - measurement_history_.at(i)) / 0.1;
//  }



//  // run one step
//  ptr_observer_->OneStepIntegration(measurement_);

  // save data
  if (step_counter_ == 1) {
    ExportData(output_directory_);
  } else {
    ExportData(output_directory_, "app");
  }

  // publish results
//  std::vector<double> observer_state = ptr_observer_->getState();
//  std::array<double, 3> observer_acc = ptr_observer_->getAcceleration();
  std::vector<double> observer_state ;
  std::array<double, 3> observer_acc ;

  QVector<double> observed;
  observed.append(current_time_);
  observed.append(measurement_.at(0));
  observed.append(measurement_.at(1));
  observed.append(measurement_.at(2));
  observed.append(observer_vel.at(0));
  observed.append(observer_vel.at(1));
  observed.append(observer_vel.at(2));
  observed.append(observer_acc.at(0));
  observed.append(observer_acc.at(1));
  observed.append(observer_acc.at(2));
  emit messageReady(observed);
//  qDebug() << observed.at(1);
//  qDebug() << observed.at(2);
//  qDebug() << observed.at(3);
//  qDebug("---- observed test ----");
}

// before motionMessageReceived ???
void ObserverInterface::controlMessageReceived(const QVector<double> &msg) {
  actuation_.at(0) = msg.at(0);
  actuation_.at(1) = msg.at(1);
  actuation_.at(2) = msg.at(2);

  ptr_observer_->setControlActuation(actuation_);
}

void ObserverInterface::ExportData(const std::string &save_directory,
                                   const std::string &mode) const {
  const std::string &observer_file =
      save_directory + "estimated_state_file.txt";

  std::ofstream state_out;

  // open the files
  if (mode == "trunc") {
    state_out.open(observer_file.c_str());
  } else {
    state_out.open(observer_file.c_str(), std::ofstream::app);
  }

  // write data to the file
  state_out << std::fixed << std::setprecision(2) << current_time_
            << '\t'; // first column is time
  state_out << std::setprecision(5);
  for (size_t index = 3; index < 15;
       ++index) { // the remaining columns are data
    state_out << std::scientific << (ptr_observer_->getState()).at(index)
              << '\t';
  }
  for (size_t i = 0; i < 3; i++) {
    state_out << std::scientific << measurement_.at(i) << '\t';
  }
  for (size_t i = 0; i < 3; i++) {
    state_out << std::scientific << (ptr_observer_->getOutput()).at(i) << '\t';
  }

  state_out << std::endl;

  // close the files
  state_out.close();
}
