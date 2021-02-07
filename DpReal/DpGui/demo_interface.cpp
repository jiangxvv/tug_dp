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

#include "demo_interface.h"
#include <cmath>

ExampleInterface::ExampleInterface(const QString &name, QWidget *parent) : QWidget(parent), topic_name_(name)
{
  example_ = new ExampleWidget();
  timer_.start(100);

  setWindowTitle("DP motion monitor");

  mainLayout_ = new QVBoxLayout;
  mainLayout_->addWidget(example_);
  setLayout(mainLayout_);

  connect(&timer_, SIGNAL(timeout()), this, SLOT(checkRos()));
  connect(this, SIGNAL(velocityChanged(const QString &)), example_->ui_->velLineEdit, SLOT(setText(const QString &)));
  connect(this, SIGNAL(headingChanged(const QString &)), example_->ui_->dirLineEdit, SLOT(setText(const QString &)));

  ros::NodeHandle node;
  sub_motion_ = node.subscribe(topic_name_.toStdString(), 100, &ExampleInterface::subCallback, this);
}

void ExampleInterface::subCallback(const gnc_msg::ObserverStamped &semi_motion)
{
  std::array<double, 3> position, velocity, acceleration;

  position.at(0) = semi_motion.position.x;
  position.at(1) = semi_motion.position.y;
  position.at(2) = semi_motion.position.z;

  velocity.at(0) = semi_motion.velocity.x;
  velocity.at(1) = semi_motion.velocity.y;
  velocity.at(2) = semi_motion.velocity.z;

  acceleration.at(0) = semi_motion.acceleration.x;
  acceleration.at(1) = semi_motion.acceleration.y;
  acceleration.at(2) = semi_motion.acceleration.z;

  double speed = std::sqrt(velocity.at(0) * velocity.at(0) + velocity.at(1) * velocity.at(1));
  double heading = position.at(2) * 180 / M_PI;

  emit velocityChanged(QString::number(speed, 'f', 3));
  emit headingChanged(QString::number(heading, 'f', 3));
}

void ExampleInterface::checkRos()
{
  ros::spinOnce();
}
