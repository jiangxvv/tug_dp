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

#include <memory>
#include "qpOASES.hpp"
#include "basic_thrust_allocation.h"
#include "dpcontroller_global.h"

USING_NAMESPACE_QPOASES

//* class SQPThrustAllocation
/**
 * SQPThrustAllocation is derived from BasicThrustAllocation.
 */
class DPCONTROLLERSHARED_EXPORT SQPThrustAllocation : public BasicThrustAllocation
{
private:
  // data for QP problem
  real_t *H_, *A_, *g_, *lb_, *ub_, *lbA_, *ubA_;

  // solution
  real_t *xOpt_;

  // QP problem with varying matrices
  std::unique_ptr<SQProblem> ptr_qproblem_;

  // parameter
  double time_step_;

public:
  /*!
 * \brief a contructor
 */
  SQPThrustAllocation(const ThrustAllocationConfiguration &config, const double &time_step);

  /*!
 * \brief a destructor
 */
  ~SQPThrustAllocation()
  {
    delete[] H_;
    delete[] A_;
    delete[] g_;
    delete[] lb_;
    delete[] ub_;
    delete[] lbA_;
    delete[] ubA_;
  }

  /*!
   * \brief return matrix B(alpha)
   * \param azimuth_angle  azimuth angle list of all the thrusters
   * \param x_coordinate   x coordinates of all the thrusters
   * \param y_coordinate   y coordinates of all the thrusters
   */
  std::array<std::vector<double>, 3> ThrustConfigMatrix(const std::vector<double> &azimuth_angle,
                                                        const std::vector<double> &x_coordinate,
                                                        const std::vector<double> &y_coordinate) const;

  /*!
   * \brief return matrix \partial{B(alpha)}/\partial{alpha}
   * \param azimuth_angle  azimuth angle list of all the thrusters
   * \param x_coordinate   x coordinates of all the thrusters
   * \param y_coordinate   y coordinates of all the thrusters
   */
  std::array<std::vector<double>, 3> ThrustConfigMatrixDerivative(const std::vector<double> &azimuth_angle,
                                                                  const std::vector<double> &x_coordinate,
                                                                  const std::vector<double> &y_coordinate) const;

  /*!
   * \brief solve first QP
   * \param tau  desired control vector
   */
  virtual int initQProblem(const std::array<double, 3> &tau);

  /*!
   * \brief solve sequential QP
   * \param tau  desired control vector
   */
  virtual int solveQProblem(const std::array<double, 3> &tau);

  /*!
   * \brief compute control output after solving QP
   * \param thruster_force  thrust force list of all the thrusters
   * \param thruster_angle  azimuth angle list of all the thrusters
   */
  void computeControlOutput(const std::vector<double> &thruster_force, const std::vector<double> &thruster_angle);

  void setH(const std::vector<double> &weights);
  void setA(const std::vector<double> &thruster_force, const std::vector<double> &thruster_angle);
  //    void setG(const std::vector<double> &thruster_force, const std::vector<double> &thruster_angle, const double &,
  //              const double &);
  void setG();
  void setLB(const std::vector<double> &thruster_force, const double &time_step);
  void setUB(const std::vector<double> &thruster_force, const double &time_step);
  void setLBA(const std::array<double, 3> &tau);
  void setUBA(const std::array<double, 3> &tau);
};
