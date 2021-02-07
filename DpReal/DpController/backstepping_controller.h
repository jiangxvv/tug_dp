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

#ifndef BACKSTEPPING_CONTROLLER_H
#define BACKSTEPPING_CONTROLLER_H

#include <Eigen>
#include <array>
#include <vector>
#include "basic_controller.h"
#include "dpcontroller_global.h"

//* class BackSteppingController
/**
 * BackSteppingController is a class derived from BasicController.
 */
class DPCONTROLLERSHARED_EXPORT BackSteppingController : public BasicController
{
private:
  // controller parameters
  Eigen::Vector3d tracking_error_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d s_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d M_ = Eigen::Matrix3d::Identity(); /**< mass matrix of the vessel */
  Eigen::Matrix3d D_ = Eigen::Matrix3d::Identity(); /**< damping matrix of the vessel */

  // static methods
  static Eigen::Matrix3d RotationMatrix(const double& angle);           //!< returns rotation matrix
  static Eigen::Matrix3d RotationMatrixTranspose(const double& angle);  //!< returns the transpose of rotation matrix
  static Eigen::Matrix3d RotationMatrixDerivative(const double& angle,
                                                  const double& rate);  //!< returns the time derivative of the rotation
                                                                        //! matrix

  // controller gain matrices
  Eigen::Matrix3d Lambda_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Kp_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Kd_ = Eigen::Matrix3d::Identity();

  // for debugging
  std::vector<double> internal_state_;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * \brief a contructor
   */
  BackSteppingController(const VesselControlProperty& vessel, const double& step_size,
                         const std::array<double, 3>& Lambda, const std::array<double, 3>& Kp,
                         const std::array<double, 3>& Kd);

  /*!
   * \brief a destructor
   */
  ~BackSteppingController()
  {
  }

  /*!
   * \brief compute control output
   */
  void ComputeControl();

  std::vector<double> getInternalState() const
  {
    return internal_state_;
  }
};

#endif  // BACKSTEPPING_CONTROLLER_H
