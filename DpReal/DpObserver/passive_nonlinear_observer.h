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

#ifndef PASSIVE_NONLINEAR_OBSERVER_H
#define PASSIVE_NONLINEAR_OBSERVER_H

#include "dpobserver_global.h"
#include <Eigen>
#include <array>
#include <boost/numeric/odeint.hpp>
#include <vector>

//* class PassiveNonlinearObserver
/**
 * Class PassiveNonlinearObserver defines an observer based on Fossen's theory.
 */
class DPOBSERVERSHARED_EXPORT PassiveNonlinearObserver {
private:
  // new matrix types
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 15, 1> Vector15d;

  // constants
  const double GRAVITY_ = 9.80665;
  const double WATER_DENSITY_ = 1025;

  // variables of the observer
  Vector6d xi_ = Vector6d::Zero();
  Eigen::Vector3d eta_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d nu_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d error_ = Eigen::Vector3d::Zero();
  std::vector<double> state_ = std::vector<double>(15, 0);

  // parameters of the observer
  std::vector<double> omega_;   /**< peak frequency of wave model */
  std::vector<double> omega_c_; /**< cutoff frequency of notch filter */
  double step_size_ = 0.1;      /**< step size used in integration */
  double lambda_ = 0.1;         /**< damping coefficient of wave model */

  Eigen::Matrix3d T_inv_ =
      Eigen::Matrix3d::Identity(); /**< inverse of time constant matrix for
                                      slowly varing load */
  Eigen::Matrix3d M_ =
      Eigen::Matrix3d::Identity(); /**< mass matrix of the vessel */
  Eigen::Matrix3d M_inv_;          /**< inverse of mass matrix of the vessel */
  Eigen::Matrix3d D_ =
      Eigen::Matrix3d::Identity(); /**< damping matrix of the vessel */
  Eigen::Matrix3d G_ = Eigen::Matrix3d::Identity(); /**< stiffness matrix of the
                                                       mooring system */
  Eigen::MatrixXd Aw_ =
      Eigen::MatrixXd::Zero(6, 6); /**< state matrix of wave model */
  Eigen::MatrixXd Cw_ = Eigen::MatrixXd::Zero(
      3, 6); /**< output matrix of wave-frequency motion */

  // observer gain matrices
  Eigen::MatrixXd K1_ = Eigen::MatrixXd::Zero(6, 3);
  Eigen::Matrix3d K2_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d K3_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d K4_ = Eigen::Matrix3d::Identity();

  // methods which return matrices
  static Eigen::Matrix3d
  RotationMatrix(const double &angle); //!< returns rotation matrix
  static Eigen::Matrix3d RotationMatrixTranspose(
      const double &angle); //!< returns the transpose of rotation matrix

  // input and ouput of the observer
  Eigen::Vector3d actuation_ = Eigen::Vector3d::Zero();
  std::vector<double> input_ = {0, 0, 0};
  std::vector<double> output_ = {0, 0, 0};

  // ode solvers
  boost::numeric::odeint::runge_kutta_dopri5<std::vector<double>> stepper_;

  Vector15d StateDerivative(
      const Vector6d &xi, const Eigen::Vector3d &eta, const Eigen::Vector3d &b,
      const Eigen::Vector3d &nu, const Eigen::Vector3d &error,
      const double &y3) const; //!< time derivative of the system state

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * \brief a constructor
   */
  PassiveNonlinearObserver(
      const std::vector<double> &mass, const std::vector<double> &added_mass,
      const std::vector<double> &damping, const std::vector<double> &stiffness,
      const std::vector<double> &k3, const std::vector<double> &k4,
      const std::vector<double> &peak_frequency,
      const std::vector<double> &cutoff_frequency, const double &T = 1000,
      const double &step_size = 0.1,
      const std::vector<double> &init_position = {0, 0, 0});

  /*!
   * \brief a destructor
   */
  ~PassiveNonlinearObserver() {}

  /*!
   * \brief system function for solving ode
   */
  void odeSystemFunction(
      const std::vector<double> &state, std::vector<double> &state_derivative,
      const double t); //!< system function for using odeint stepper

  /*!
   * \brief one step integration
   */
  void OneStepIntegration(const std::vector<double> &measurement);

  void updateOuput() {
    Eigen::Vector3d out = eta_ + Cw_ * xi_;
    for (size_t i = 0; i < 3; i++) {
      output_.at(i) = out(i);
    }
  }

  void setControlActuation(std::array<double, 3> input) {
    for (size_t i = 0; i < 3; i++) {
      actuation_(i) = input.at(i);
    }
  }

  /*!
   * \brief return observer states
   */
  std::vector<double> getState() { return state_; }

  /*!
   * \brief return estimate error
   */
  std::vector<double> getOutput() { return output_; }

  /*!
   * \brief return estimated acceleration
   */
  std::array<double, 3> getAcceleration();
};

#endif // PASSIVE_NONLINEAR_OBSERVER_H
