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

#ifndef OBSERVER_INTERFACE_H
#define OBSERVER_INTERFACE_H

#include <QObject>
#include <QVector>
#include <memory>
#include <passive_nonlinear_observer.h>
#include <string>
#include <vector>
#include <alpha_belta_filter.h>
#include <Eigen>

/*!
 * \brief The VesselObserverProperty struct
 */
struct VesselObserverProperty {
  std::string name;
  std::vector<double> mass;
  std::vector<double> added_mass;
  std::vector<double> damping;
  std::vector<double> stiffness;
};

//* class ObserverInterface
/**
 * The class defines a time domain simulation interface for a passive nonlinear
 * observer.
 */
class ObserverInterface : public QObject {
  Q_OBJECT

private:
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
  std::unique_ptr<PassiveNonlinearObserver> ptr_observer_;
  std::unique_ptr<AlphaBeltaFilter> ptr_alpha_belta_filter_;

  // measurements
  double current_time_ = 0.0;
  size_t step_counter_ = 0;
  std::vector<double> measurement_ = {0, 0, 0};
  std::vector<double> measurement_history_ = {0, 0, 0};
  std::array<double, 3> actuation_;
  Vector6d QualysisInfo;



  std::string output_directory_;

signals:
  /*!
   * \brief messageReady
   * \param msg
   */
  void messageReady(const QVector<double> &msg);

public slots:

  /*!
   * \brief motionMessageReceived
   * \param msg motion message passed by simulator
   */
  void motionMessageReceived(const QVector<double> &msg);

  /*!
   * \brief controlMessageReceived
   * \param msg actuation message passed by controller
   */
  void controlMessageReceived(const QVector<double> &msg);

public:
  /*!
   * \brief a constructor
   */
  ObserverInterface(const VesselObserverProperty &vessel,
                    const double &step_size,
                    const std::vector<double> &init_position,
                    std::string save_path);

  /*!
   * \brief ExportData
   * \param save_directory
   * \param mode
   */
  void ExportData(const std::string &save_directory,
                  const std::string &mode = "trunc") const;
};

#endif // OBSERVER_INTERFACE_H
