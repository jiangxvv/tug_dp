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

#include "basic_thrust_allocation.h"

BasicThrustAllocation::BasicThrustAllocation(const ThrustAllocationConfiguration& config)
{
  azimuth_thruster_number_ = config.azimuth_thruster_number;
  tunnel_thruster_number_ = config.tunnel_thruster_number;
  main_thruster_number_ = config.main_thruster_number;
  total_thruster_number_ = azimuth_thruster_number_ + tunnel_thruster_number_ + main_thruster_number_;

  x_coordinates_ = config.x_coordinates;
  y_coordinates_ = config.y_coordinates;

  thrust_upper_limit_ = config.thrust_upper_limit;
  thrust_lower_limit_ = config.thrust_lower_limit;
  thrust_rate_limit_ = config.thrust_rate_limit;
  rotation_rate_limit_ = config.rotation_rate_limit;

  for (size_t i = 0; i < total_thruster_number_; ++i)
  {
    thruster_angle_.push_back(0);
    thruster_force_.push_back(0);
  }
}
