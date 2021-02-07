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

#ifndef BASIC_CONTROLLER_H
#define BASIC_CONTROLLER_H

#include <array>
#include <string>
#include "dpcontroller_global.h"

struct DPCONTROLLERSHARED_EXPORT VesselControlProperty
{
  std::string name;
  double mass = 0;
  double Izz = 0;
  double a11 = 0;
  double a22 = 0;
  double a23 = 0;
  double a33 = 0;
  double xg = 0;
  double d11 = 0;
  double d22 = 0;
  double d33 = 0;
};

//* class BasicController
/**
 * BasicController is a base class, which defines the basic controller structures.
 */
class DPCONTROLLERSHARED_EXPORT BasicController
{
protected:
  // vessel parameters
  const double GRAVITY_ = 9.81;
  double mass_ = 0; /**< vessel mass */
  double Izz_ = 0;  /**< inertia moment */
  double a11_ = 0;  /**< added mass in surge */
  double a22_ = 0;  /**< added mass in sway */
  double a23_ = 0;  /**< added mass cross term */
  double a33_ = 0;  /**< added mass in yaw */
  double xg_ = 0;   /**< longitudinal coordinate of gravity center */
  double d11_ = 0;  /**< linear damping coefficient in surge */
  double d22_ = 0;  /**< linear damping coefficient in sway */
  double d33_ = 0;  /**< linear damping coefficient in yaw */

  // vessel states
  std::array<double, 3> vessel_position_ = { { 0, 0, 0 } };     /**< vessel coordinate and heading */
  std::array<double, 3> vessel_velocity_ = { { 0, 0, 0 } };     /**< vessel linear velocity and yaw rate */
  std::array<double, 3> vessel_acceleration_ = { { 0, 0, 0 } }; /**< vessel acceleration */

  // environmental conditions
  double wind_angle_ = 0;       /**< wind angle with respect to the body frame */
  double current_angle_ = 0;    /**< current angle with respect to the body frame */
  double wave_angle_ = 0;       /**< wave propagation angle with respect to the body frame */
  double wind_velocity_ = 0;    /**< wind velocity */
  double current_velocity_ = 0; /**< current velocity */

  // control variables
  double step_size_ = 0.0;
  std::array<double, 3> position_setpoint_ = { { 0, 0, 0 } }; /**< position setpoint in x, y, and heading */
  std::array<double, 3> velocity_setpoint_ = { { 0, 0, 0 } }; /**< velocity setpoint in x, y, and heading */
  std::array<double, 3> position_error_ = { { 0, 0, 0 } };    /**< position error in x, y, and heading */
  std::array<double, 3> output_ = { { 0, 0, 0 } };            /**< control output in surge, sway, and yaw */

public:
  /*!
   * \brief a contructor
   */
  BasicController(const VesselControlProperty& vessel, const double& step_size);

  /*!
   * \brief a destructor
   */
  ~BasicController()
  {
  }

  /*!
   * \brief return control output
   */
  std::array<double, 3> getOutput() const
  {
    return output_;
  }

  /*!
   * \brief set vessel position setpoint in the earth-fixed reference frame
   */
  void setPositionSetpoint(const std::array<double, 3>& input)
  {
    position_setpoint_ = input;
  }

  /*!
   * \brief set vessel velocity setpoint in the earth0-fixed reference frame
   */
  void setVelocitySetpoint(const std::array<double, 3>& input)
  {
    velocity_setpoint_ = input;
  }

  /*!
   * \brief set vessel position
   */
  void setVesselPosition(const std::array<double, 3>& input)
  {
    vessel_position_ = input;

    for (size_t i = 0; i < 3; i++)
    {
      position_error_.at(i) = input.at(i) - position_setpoint_.at(i);
    }
  }

  /*!
   * \brief set vessel velocity
   */
  void setVesselVelocity(const std::array<double, 3>& input)
  {
    vessel_velocity_ = input;
  }

  /*!
   * \brief set vessel acceleration
   */
  void setVesselAcceleration(const std::array<double, 3>& input)
  {
    vessel_acceleration_ = input;
  }

  /*!
   * \brief return time step size of the controller
   */
  double getStepSize() const
  {
    return step_size_;
  }
};

#endif  // BASIC_CONTROLLER_H
