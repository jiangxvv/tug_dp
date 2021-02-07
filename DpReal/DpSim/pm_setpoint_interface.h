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

#ifndef PM_SETPOINT_INTERFACE
#define PM_SETPOINT_INTERFACE

//#include <actionlib/server/simple_action_server.h>
//#include <gnc_msg/PmRadiusSetpointAction.h>
#include <string>

//* class PmSetpointInterface
/**
 * The class defines an interface with the generator of the optimal radius setpoint.
 */
class PmSetpointInterface
{
private:
  actionlib::SimpleActionServer<gnc_msg::PmRadiusSetpointAction> as_;

  // parameters
  double radius_setpoint_ = 1.0;
  double time_window_ = 600;
  double time_step_ = 0.0;
  signed int data_size_;
  signed int size_counter_ = 0;

  // radius
  double current_radius_ = 0.0;
  double mean_radius_ = 0.0;

  // power level
  double current_power_ = 0.0;
  double mean_power_ = 0.0;

public:
  /*!
   * \brief a constructor
   */
  PmSetpointInterface(const std::string &server_name, const double &initial_setpoint, const double &time_window,
                      const double &step_size)
    : as_(server_name, false), radius_setpoint_(initial_setpoint), time_window_(time_window), time_step_(step_size)
  {
    as_.registerGoalCallback(boost::bind(&PmSetpointInterface::goalCallback, this));
    as_.start();

    data_size_ = int(time_window_/time_step_);
  }

  /*!
   * \brief set the radius setpoint
   */
  void setRadiusSetpoint(const double &value)
  {
    radius_setpoint_ = value;
  }

  /*!
   * \brief get the radius setpoint
   */
  double getRadiusSetpoint() const
  {
    return radius_setpoint_;
  }

  /*!
   * \brief update radius and power
   */
  void updateRadiusAndPower(const double &radius, const double &power);

  void goalCallback();
};

#endif  // PM_SETPOINT_INTERFACE
