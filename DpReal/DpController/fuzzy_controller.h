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

#ifndef FUZZY_CONTROLLER_H
#define FUZZY_CONTROLLER_H

#include <array>
#include <deque>
#include <vector>
#include "basic_controller.h"
#include "dpcontroller_global.h"

//* struct ControlFuzzySet
/**
 * ControlFuzzySet defines the fuzzy set for the control output.
 */
struct DPCONTROLLERSHARED_EXPORT ControlFuzzySet
{
  const double PL = 1.000;
  const double PB = 0.750;
  const double PM = 0.500;
  const double PS = 0.250;
  const double PZ = 0.125;
  const double ZO = 0.000;
  const double NZ = -0.125;
  const double NS = -0.250;
  const double NM = -0.500;
  const double NB = -0.750;
  const double NL = -1.000;
};

struct DPCONTROLLERSHARED_EXPORT PositionInputFuzzySet
{
  const double NB = -0.833;
  const double NM = -0.500;
  const double NS = -0.167;
  const double PS = 0.167;
  const double PM = 0.500;
  const double PB = 0.833;
};

struct DPCONTROLLERSHARED_EXPORT VelocityInputFuzzySet
{
  const double NB = -0.8571;
  const double NM = -0.5714;
  const double NS = -0.2857;
  const double ZO = 0.00;
  const double PS = 0.2857;
  const double PM = 0.5714;
  const double PB = 0.8571;
};

struct DPCONTROLLERSHARED_EXPORT StaticOutputFuzzySet
{
  const double NL = -7.0E4;
  const double NB = -5.0E4;
  const double NM = -3.0E4;
  const double NS = -1.0E4;
  const double NZ = -0.4E4;
  const double PZ = 0.4E4;
  const double PS = 1.0E4;
  const double PM = 3.0E4;
  const double PB = 5.0E4;
  const double PL = 7.0E4;
};

//* class FuzzyController
/**
 * FuzzyController is a class derived from BasicController.
 */
class DPCONTROLLERSHARED_EXPORT FuzzyController : public BasicController
{
private:
  // controller parameters
  ControlFuzzySet control_fuzzy_;
  PositionInputFuzzySet position_fuzzy_;
  VelocityInputFuzzySet velocity_fuzzy_;
  StaticOutputFuzzySet static_output_fuzzy_;
  std::array<std::array<double, 7>, 8> fuzzy_form_;                 /**< fuzzy control table */
  std::array<double, 3> Ke_ = { { 6, 6, 12 } };                     /**< qualification factor for position error */
  std::array<double, 3> Kec_ = { { 0.35, 0.35, 3.5 } };             /**< qualification factor for vessel velocity */
  double Ku_ = 0.1;                                                 /**< proportional factor */
  std::array<double, 5> static_range_ = { { 0.5, 1.5, 3, 5, 10 } }; /**< static range used in static control */
  unsigned int static_control_size_ = 50; /**< step interval between two static control output */

  // variables
  std::array<unsigned int, 3> E_ = { { 0, 0, 0 } };       /**< error set */
  std::array<unsigned int, 3> EC_ = { { 0, 0, 0 } };      /**< error rate set */
  std::array<std::deque<double>, 3> position_memory_;     /**< store vessel position over a certain time range */
  std::array<double, 3> mean_position_ = { { 0, 0, 0 } }; /**< mean vessel position in a time range */

  // output factors
  std::array<double, 3> dynamic_factor_ = { { 1, 1, 1 } };
  std::array<double, 3> static_factor_ = { { 1, 1, 1 } };

  // control outputs
  std::array<double, 3> dynamic_output_ = { { 0, 0, 0 } };
  std::array<double, 3> static_output_ = { { 0, 0, 0 } };
  std::array<double, 3> fuzzy_output_ = { { 0, 0, 0 } };

public:
  /*!
   * \brief a contructor
   */
  FuzzyController(const VesselControlProperty& vessel, const double& step_size, const unsigned int& static_control_length);

  /*!
   * \brief a destructor
   */
  ~FuzzyController()
  {
  }

  /*!
   * \brief convert position error and vessel velocity to E and EC
   */
  void Fuzzification();

  /*!
   * \brief use E and EC to compute dynamic control output
   * \param heading  vessel heading angle
   */
  void DynamicControl(const double& heading);

  /*!
   * \brief compute static control output
   * \param velocity        vessel velocity
   * \param prior_velocity  vessel velocity at the last time step
   * \param mean            mean vessel position over a time range
   * \param increment       increment to the ouput
   * \param output          static output
   */
  void StaticControl(const double& velocity, const double& prior_velocity, const double& mean, double& increment,
                     double& output);

  /*!
   * \brief compute total control output
   */
  void FuzzyControl();

  /*!
   * \brief update position memory and mean position
   * \param new_position new vessel position
   */
  void ComputeMeanPosition(const std::array<double, 3>& new_position);

  /*!
   * \brief return total control output
   */
  std::array<double, 3> getControlOutput()
  {
    return fuzzy_output_;
  }
  //控制参数的接口 回头写
//  void setK(const double& k)
//  {
//      ke = k;

//  }
};

#endif  // FUZZY_CONTROLLER_H
