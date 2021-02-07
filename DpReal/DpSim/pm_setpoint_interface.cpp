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

#include "pm_setpoint_interface.h"
#include <iostream>

void PmSetpointInterface::goalCallback()
{
  // get the goal
  auto ptr_goal = as_.acceptNewGoal();
  std::cout << "A new goal is received, and the setpoint is " << ptr_goal->setpoint << "m." << std::endl;

  radius_setpoint_ = ptr_goal->setpoint;
}

void PmSetpointInterface::updateRadiusAndPower(const double &radius, const double &power)
{
  current_radius_ = radius;
  current_power_ = power;

  mean_radius_ += current_radius_ / data_size_;
  mean_power_ += current_power_ / data_size_;

  if (size_counter_ < data_size_)
  {
    ++size_counter_;
  }
  else
  {
    gnc_msg::PmRadiusSetpointResult result;
    result.power_consumption = mean_power_;
    result.radius = mean_radius_;
    as_.setSucceeded(result, "Session completed");

    // gnc_msg::PmRadiusSetpointFeedback feedback;  // for debug only
    // feedback.power_consumption = mean_power_;
    // feedback.radius = mean_radius_;
    // as_.publishFeedback(feedback);
    
    // reset the data
    size_counter_ = 0;
    mean_power_ = 0;
    mean_radius_ = 0;
  }
}