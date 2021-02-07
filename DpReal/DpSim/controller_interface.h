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

#ifndef CONTROLLER_INTERFACE_H
#define CONTROLLER_INTERFACE_H

#include <QObject>
#include <QVector>
#include <backstepping_controller.h>
#include <pid_controller.h>
#include <iostream>
#include <memory.h>
#include <pm_controller.h>
#include <sqp_thrust_allocation.h>
#include <inverse_thrust_allocation.h>
#include <Eigen>
#include <fuzzy_controller.h>

typedef Eigen::Matrix<double, 3, 2> Matrix32;

//* class ControllerInterface
/**
 * The class defines an interface for the controller.
 */
class ControllerInterface : public QObject {
  Q_OBJECT

private:
  // controllers
  //std::unique_ptr<PmController> ptr_controller_;
  //    std::unique_ptr<PmSetpointInterface> ptr_pm_setpoint_interface_;
  //    std::unique_ptr<BackSteppingController> ptr_controller_;

  //动力定位试验 fuzzycontroller
//  std::unique_ptr<FuzzyController> ptr_controller_;

  std::unique_ptr<PIDController> ptr_controller_;

  std::unique_ptr<inverse_thrust_allocation> ptr_allocator_;

  Matrix32 L;

  Eigen::Vector3d output_for_allocator;

  std::vector<double> controller_state_;


   // parameter
  double step_size_ = 0.0;         /**< time step size of simulation */
  unsigned int step_interval_ = 0; /**< step interval for the controller */

  // thruster system
  int qp_counter_ = 0;
  std::string output_directory_;
  std::vector<double> thruster_force_;
  std::vector<double> thruster_angle_;
  std::array<double, 3> control_output_;
  std::array<double, 3> allocation_output_;

signals:
  void messageReady(const QVector<double> &msg);
  void thrusterReady(const QVector<double> &msg);

public slots:
  /*!
   * \brief messageReceived
   * \param msg observer message passsed by ObserverSubscriber
   */
  void messageReceived(const QVector<double> &msg);
  void setpointReceived(const QVector<double> &msg);

public:
  /*!
   * \brief a constructor
   */
  ControllerInterface(const VesselControlProperty &vessel,
                      const ThrustAllocationConfiguration &config,
                      const std::array<double, 3> &PositionSetPoint,
                      const double &step_size, std::string save_path);
  /*!
   * \brief a destructor
   */
  ~ControllerInterface() override = default;

  /*!
   * \brief save thruster data to file
   */
  void ExportData(const std::string &save_directory,
                  const std::string &mode = "trunc") const;
};

#endif // CONTROLLER_INTERFACE_H
