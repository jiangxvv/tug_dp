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

#include "fuzzy_controller.h"
#include <cmath>
#include <cstdio>
#include <numeric>

FuzzyController::FuzzyController(const VesselControlProperty& vessel, const double& step_size,
                                 const unsigned int& static_control_length)
  : BasicController(vessel, step_size), static_control_size_(static_control_length)
{
  // fuzzy control form
  fuzzy_form_ = { { { control_fuzzy_.PL, control_fuzzy_.PL, control_fuzzy_.PM, control_fuzzy_.PM, control_fuzzy_.PS,
                      control_fuzzy_.ZO, control_fuzzy_.ZO },
                    { control_fuzzy_.PL, control_fuzzy_.PB, control_fuzzy_.PM, control_fuzzy_.PM, control_fuzzy_.PS,
                      control_fuzzy_.ZO, control_fuzzy_.ZO },
                    { control_fuzzy_.PM, control_fuzzy_.PM, control_fuzzy_.PM, control_fuzzy_.PS, control_fuzzy_.ZO,
                      control_fuzzy_.NM, control_fuzzy_.NM },
                    { control_fuzzy_.PM, control_fuzzy_.PS, control_fuzzy_.PZ, control_fuzzy_.ZO, control_fuzzy_.NZ,
                      control_fuzzy_.NS, control_fuzzy_.NM },
                    { control_fuzzy_.PM, control_fuzzy_.PS, control_fuzzy_.PZ, control_fuzzy_.ZO, control_fuzzy_.NZ,
                      control_fuzzy_.NS, control_fuzzy_.NM },
                    { control_fuzzy_.PM, control_fuzzy_.PS, control_fuzzy_.ZO, control_fuzzy_.NS, control_fuzzy_.NM,
                      control_fuzzy_.NB, control_fuzzy_.NB },
                    { control_fuzzy_.ZO, control_fuzzy_.ZO, control_fuzzy_.NS, control_fuzzy_.NM, control_fuzzy_.NM,
                      control_fuzzy_.NB, control_fuzzy_.NL },
                    { control_fuzzy_.ZO, control_fuzzy_.ZO, control_fuzzy_.NS, control_fuzzy_.NM, control_fuzzy_.NM,
                      control_fuzzy_.NL, control_fuzzy_.NL } } };
}

void FuzzyController::Fuzzification()
{
  std::array<double, 3> position;
  std::array<double, 3> velocity;

  position.at(0) = position_error_.at(0) / Ke_.at(0);
  position.at(1) = position_error_.at(1) / Ke_.at(1);
  position.at(2) = position_error_.at(2) * (180 / M_PI) / Ke_.at(2);
  velocity.at(0) = vessel_velocity_.at(0) / Kec_.at(0);
  velocity.at(1) = vessel_velocity_.at(1) / Kec_.at(1);
  velocity.at(2) = vessel_velocity_.at(2) * (180 / M_PI) / Kec_.at(2);

  for (size_t i = 0; i < 3; i++)
  {
    // position error
    if (position.at(i) <= position_fuzzy_.NB)
      E_.at(i) = 0;
    else if (position.at(i) <= position_fuzzy_.NM)
      E_.at(i) = 1;
    else if (position.at(i) <= position_fuzzy_.NS)
      E_.at(i) = 2;
    else if (position.at(i) <= position_fuzzy_.PS)
      E_.at(i) = 3;
    else if (position.at(i) <= position_fuzzy_.PM)
      E_.at(i) = 4;
    else if (position.at(i) <= position_fuzzy_.PB)
      E_.at(i) = 5;
    else
      E_.at(i) = 6;

    // position rate
    if (velocity.at(i) <= velocity_fuzzy_.NB)
      EC_.at(i) = 0;
    else if (velocity.at(i) <= velocity_fuzzy_.NM)
      EC_.at(i) = 1;
    else if (velocity.at(i) <= velocity_fuzzy_.NS)
      EC_.at(i) = 2;
    else if (velocity.at(i) <= velocity_fuzzy_.ZO)
      EC_.at(i) = 3;
    else if (velocity.at(i) <= velocity_fuzzy_.PS)
      EC_.at(i) = 4;
    else if (velocity.at(i) <= velocity_fuzzy_.PM)
      EC_.at(i) = 5;
    else if (velocity.at(i) <= velocity_fuzzy_.PB)
      EC_.at(i) = 6;
    else
      EC_.at(i) = 7;
  }

  printf("E: %1i, %1i, %1i, and EC: %1i, %1i, %1i\n", E_.at(0), E_.at(1), E_.at(2), EC_.at(0), EC_.at(1), EC_.at(2));
}

void FuzzyController::DynamicControl(const double& heading)
{
  // accelerations in the earth-fixed reference frame
  double u_acc = Ku_ * (fuzzy_form_.at(EC_.at(0))).at(E_.at(0));
  double v_acc = Ku_ * (fuzzy_form_.at(EC_.at(1))).at(E_.at(1));
  double r_acc = Ku_ * (fuzzy_form_.at(EC_.at(2))).at(E_.at(2)) * 1E-2;

  // compute actuations in the body-fixed reference frame
  dynamic_output_.at(0) = (mass_ + a11_) * (u_acc * std::cos(heading) + v_acc * std::sin(heading));
  dynamic_output_.at(1) = (mass_ + a22_) * (-u_acc * std::sin(heading) + v_acc * std::cos(heading));
  dynamic_output_.at(2) = (Izz_ + a33_) * r_acc;
}

void FuzzyController::StaticControl(const double& velocity, const double& prior_velocity, const double& mean,
                                    double& increment, double& output)
{
  if (mean < -static_range_.at(4) && velocity <= prior_velocity)
  {
    increment = static_output_fuzzy_.PL;
    output += increment;
  }
  else if (mean < -static_range_.at(3) && velocity <= prior_velocity)
  {
    if (increment >= static_output_fuzzy_.PB)
    {
      increment = static_output_fuzzy_.PB;
      return;
    }
    increment = static_output_fuzzy_.PB;
    output += increment;
  }
  else if (mean < -static_range_.at(2) && velocity <= prior_velocity)
  {
    if (increment >= static_output_fuzzy_.PM)
    {
      increment = static_output_fuzzy_.PM;
      return;
    }
    increment = static_output_fuzzy_.PM;
    output += increment;
  }
  else if (mean < -static_range_.at(1) && velocity <= prior_velocity)
  {
    if (increment >= static_output_fuzzy_.PS)
    {
      increment = static_output_fuzzy_.PS;
      return;
    }
    increment = static_output_fuzzy_.PS;
    output += increment;
  }
  else if (mean < -static_range_.at(0) && velocity <= prior_velocity)
  {
    if (increment >= static_output_fuzzy_.PZ)
    {
      increment = static_output_fuzzy_.PZ;
      return;
    }
    increment = static_output_fuzzy_.PZ;
    output += increment;
  }
  else if (mean < static_range_.at(0))
  {
    output += 0.0;
  }
  else if (mean < static_range_.at(1) && velocity >= prior_velocity)
  {
    if (increment <= static_output_fuzzy_.NZ)
    {
      increment = static_output_fuzzy_.NZ;
      return;
    }
    increment = static_output_fuzzy_.NZ;
    output += increment;
  }
  else if (mean < static_range_.at(2) && velocity >= prior_velocity)
  {
    if (increment <= static_output_fuzzy_.NS)
    {
      increment = static_output_fuzzy_.NS;
      return;
    }
    increment = static_output_fuzzy_.NS;
    output += increment;
  }
  else if (mean < static_range_.at(3) && velocity >= prior_velocity)
  {
    if (increment <= static_output_fuzzy_.NM)
    {
      increment = static_output_fuzzy_.NM;
      return;
    }
    increment = static_output_fuzzy_.NM;
    output += increment;
  }
  else if (mean < static_range_.at(4) && velocity >= prior_velocity)
  {
    if (increment <= static_output_fuzzy_.NB)
    {
      increment = static_output_fuzzy_.NB;
      return;
    }
    increment = static_output_fuzzy_.NB;
    output += increment;
  }
  else if (mean >= static_range_.at(4) && velocity >= prior_velocity)
  {
    increment = static_output_fuzzy_.NL;
    output += increment;
  }
}

void FuzzyController::FuzzyControl()
{
  static unsigned int counter = 0;
  static std::array<double, 3> prior_velocity = { { 0, 0, 0 } };
  static std::array<double, 3> prior_increment = { { 0, 0, 0 } };

  ++counter;
  if (counter >= static_control_size_)
  {
    // control force
    StaticControl(vessel_velocity_.at(0), prior_velocity.at(0), mean_position_.at(0), prior_increment.at(0),
                  static_output_.at(0));
    StaticControl(vessel_velocity_.at(1), prior_velocity.at(1), mean_position_.at(1), prior_increment.at(1),
                  static_output_.at(1));

    if (mean_position_.at(2) > M_PI / 6 || mean_position_.at(2) < -M_PI / 6)
    {
      static_output_.at(0) *= 0.9;
      static_output_.at(1) *= 0.9;
    }

    // control moment
    StaticControl(vessel_velocity_.at(2), prior_velocity.at(2), mean_position_.at(2) * 180 / M_PI,
                  prior_increment.at(2), static_output_.at(2));

    // deal with conflict
    for (size_t i = 0; i < 2; i++)
    {
      if (static_output_.at(i) * dynamic_output_.at(i) < 0)
      {
        if (std::fabs(position_error_.at(i)) > 20.0)
          static_output_.at(i) *= 0.7;
        else
          static_output_.at(i) *= 0.95;
      }
    }

    if (static_output_.at(2) * dynamic_output_.at(2) < 0)
    {
      if (std::fabs(position_error_.at(2)) > M_PI / 18)
        static_output_.at(2) *= 0.7;
      else
        static_output_.at(2) *= 0.95;
    }

    // store new mean velocity
    prior_velocity.at(0) = vessel_velocity_.at(0);
    prior_velocity.at(1) = vessel_velocity_.at(1);
    prior_velocity.at(2) = vessel_velocity_.at(2);

    // reset counter
    counter = 0;
  }

  // transform static control output to the body-fixed reference frame
  double static_x = static_output_.at(0);
  double static_y = static_output_.at(1);
  static_output_.at(0) = static_x * std::cos(vessel_position_.at(2)) + static_y * std::sin(vessel_position_.at(2));
  static_output_.at(1) = -static_x * std::sin(vessel_position_.at(2)) + static_y * std::cos(vessel_position_.at(2));

  // dynamic control
  Fuzzification();
  DynamicControl(vessel_position_.at(2));

  // compute the total control output
  for (size_t i = 0; i < 3; i++)
  {
    fuzzy_output_.at(i) = dynamic_output_.at(i) * dynamic_factor_.at(i) + static_output_.at(i) * static_factor_.at(i);
  }

  printf("dynamic: %8.2f, %8.2f, %8.2f, and static: %8.2f, %8.2f, %8.2f\n", dynamic_output_.at(0) / 1E3,
         dynamic_output_.at(1) / 1E3, dynamic_output_.at(2) / 1E3, static_output_.at(0) / 1E3,
         static_output_.at(1) / 1E3, static_output_.at(2) / 1E3);
}

void FuzzyController::ComputeMeanPosition(const std::array<double, 3>& new_position)
{
  for (size_t i = 0; i < 3; i++)
  {
    position_memory_.at(i).push_back(new_position.at(i));

    if (position_memory_.at(i).size() > static_control_size_)
    {
      position_memory_.at(i).pop_front();
    }

    double sum = std::accumulate(position_memory_.at(i).begin(), position_memory_.at(i).end(), 0);
    mean_position_.at(i) = sum / position_memory_.at(i).size();
  }
  printf("--------------------------------------\n");
  printf("mean position: %5.2f, %5.2f, %5.2f\n", mean_position_.at(0), mean_position_.at(1),
         mean_position_.at(2) * 57.3);
}
