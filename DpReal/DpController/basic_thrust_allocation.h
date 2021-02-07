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

#include <array>
#include <vector>
#include "dpcontroller_global.h"

//* Struct ThrustAllocationParameter
/**
 * ThrustAllocationParameter defines the parameters used to construct a thrust allocation instance.
 */
struct DPCONTROLLERSHARED_EXPORT ThrustAllocationConfiguration
{
  unsigned int azimuth_thruster_number = 0;
  unsigned int tunnel_thruster_number = 0;
  unsigned int main_thruster_number = 0;
  std::vector<double> x_coordinates;
  std::vector<double> y_coordinates;
  std::vector<double> thrust_upper_limit;  // for azimuth, tunnel, and main thruster
  std::vector<double> thrust_lower_limit;
  std::vector<double> thrust_rate_limit;
  std::vector<double> rotation_rate_limit;
  std::vector<double> weights;
  std::vector<double> initial_thrust;
  std::vector<double> initial_azimuth;
};

//* class BasicThrustAllocation
/**
 * BasicThrustAllocation is a base class, which defines the basic structures for thrust allocation algorithm.
 */
class DPCONTROLLERSHARED_EXPORT BasicThrustAllocation
{
protected:
  unsigned int azimuth_thruster_number_;
  unsigned int tunnel_thruster_number_;
  unsigned int main_thruster_number_;
  unsigned int total_thruster_number_;

  // thruster coordinates in the body-fixed reference frame
  std::vector<double> x_coordinates_;
  std::vector<double> y_coordinates_;

  std::vector<double> thrust_upper_limit_;
  std::vector<double> thrust_lower_limit_;
  std::vector<double> thrust_rate_limit_;
  std::vector<double> rotation_rate_limit_; /**< maximal rotational rate */

  std::vector<double> thruster_angle_;   /**< angle list of all the thruster */
  std::vector<double> thruster_force_;   /**< thrust list of all the thruster */
  std::array<double, 3> control_output_; /**< resultant control force */
  std::array<double, 3> error_;          /**< difference between desired and allocated thrust */

  /*!
   * \brief a contructor
   */
  BasicThrustAllocation(const ThrustAllocationConfiguration &config);

public:
  /*!
   * \brief compute control output
   */
  virtual void computeControlOutput(const std::vector<double> &, const std::vector<double> &) = 0;

  /*!
   * \brief return all the thruster angles
   */
  std::vector<double> getThrusterAngle() const
  {
    return thruster_angle_;
  }

  /*!
   * \brief return all the thruster forces
   */
  std::vector<double> getThrusterForce() const
  {
    return thruster_force_;
  }

  /*!
   * \brief return thruster angle given an index
   */
  double getThrusterAngle(const unsigned int &idx) const
  {
    return thruster_angle_.at(idx);
  }

  /*!
   * \brief return thrust force given an index
   */
  double getThrusterForce(const unsigned int &idx) const
  {
    return thruster_force_.at(idx);
  }

  /*!
   * \brief set all the thruster angles
   */
  void setThrusterAngle(const std::vector<double> &value)
  {
    thruster_angle_ = value;
  }

  /*!
   * \brief set all the thruster forces
   */
  void setThrusterForce(const std::vector<double> &value)
  {
    thruster_force_ = value;
  }

  /*!
   * \brief set thruster angle at an index
   */
  void setThrusterAngle(const unsigned int &idx, const double &value)
  {
    thruster_angle_.at(idx) = value;
  }

  /*!
   * \brief set thrust force at an index
   */
  void setThrusterForce(const unsigned int &idx, const double &value)
  {
    thruster_force_.at(idx) = value;
  }

  /*!
   * \brief return resultant control output
   */
  std::array<double, 3> getControlOutput() const
  {
    return control_output_;
  }
};
