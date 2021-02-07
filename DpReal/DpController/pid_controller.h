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

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <array>
#include "basic_controller.h"
#include "dpcontroller_global.h"

//* class PIDController
/**
 * PIDControlleris a class derived from BasicController.
 */
class DPCONTROLLERSHARED_EXPORT PIDController : public BasicController
{
private:
  // pid gains
  std::array<double, 3> Kp_ = { { 0, 0, 0 } };
  std::array<double, 3> Ki_ = { { 0, 0, 0 } };
  std::array<double, 3> Kd_ = { { 0, 0, 0 } };

public:
  /*!
   * \brief a contructor
   */
  PIDController(const VesselControlProperty& vessel, const double& step_size, const std::array<double, 3>& Kp,
                const std::array<double, 3>& Kd);

  /*!
   * \brief a destructor
   */
  ~PIDController()
  {
  }

  /*!
   * \brief compute control output
   */
  void ComputeControl();

  /*!
   * \brief return P gains
   */
  std::array<double, 3> getPGain() const
  {
    return Kp_;
  }

  /*!
   * \brief return D gains
   */
  std::array<double, 3> getDGain() const
  {
    return Kd_;
  }

  /*!
   * \brief set P gains
   */
  void setPGain(const std::array<double, 3>& input)
  {
    Kp_ = input;
  }

  /*!
   * \brief set D gains
   */
  void setDGain(const std::array<double, 3>& input)
  {
    Kd_ = input;
  }
};

#endif  // PID_CONTROLLER_H
