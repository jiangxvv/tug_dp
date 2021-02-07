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

#include "core_window.h"
#include "polar_plot.h"
#include "trajectory.h"
#include <QGridLayout>
#include <QtMath>
#include <QMetaType>
#include <QVBoxLayout>
#include<QHBoxLayout>


CoreWindow::CoreWindow(QWidget *parent) : QWidget(parent) {
  polarPlot = new PolarPlot;
  trajectory = new Trajectory;

  QGridLayout *gridLayout = new QGridLayout;
  gridLayout->addWidget(polarPlot, 0, 0);
  gridLayout->addWidget(trajectory, 0, 0);
  QVBoxLayout *Vlayout = new QVBoxLayout;
  Vlayout->addWidget(setpointLable);
  Vlayout->addWidget(setpointEdit1);
  Vlayout->addWidget(setpointEdit2);
  Vlayout->addWidget(setpointEdit3);
  Vlayout->addWidget(kpLabel);
  Vlayout->addWidget(kpEdit1);
  Vlayout->addWidget(kpEdit2);
  Vlayout->addWidget(kpEdit3);
  Vlayout->addWidget(kdLabel);
  Vlayout->addWidget(kdEdit1);
  Vlayout->addWidget(kdEdit2);
  Vlayout->addWidget(kdEdit3);
  Vlayout->addWidget(confirmButton);
  Vlayout->addWidget(closeButton);
  QHBoxLayout *Hlayout = new QHBoxLayout;
  Hlayout->addLayout(gridLayout);
  Hlayout->addLayout(Vlayout);
  Hlayout->setStretchFactor(gridLayout, 4);
  Hlayout->setStretchFactor(Vlayout, 1);
  setLayout(Hlayout);

  setWindowTitle("DP Motion Monitor");
  setGeometry(100, 100, 1000, 1000);

  connect(this, SIGNAL(poseChanged(const QVector<double> &)), trajectory,
          SLOT(updatePosition(const QVector<double> &)));
  connect(confirmButton, &QPushButton::clicked, this, &CoreWindow::setpointSelect);
  connect(closeButton, QPushButton::clicked, this, &CoreWindow::servo_close);
}

void CoreWindow::messageReceived(const QVector<double> &msg) {
  std::array<double, 3> position;

  double scale_ratio = getScaleRatio();

  position.at(0) = msg.at(1) * scale_ratio;
  position.at(1) = msg.at(2) * scale_ratio;
  position.at(2) = qRadiansToDegrees(msg.at(3));

  QVector<double> pose = {position.at(0), position.at(1), position.at(2)};

  emit poseChanged(pose);
}

void CoreWindow::setpointSelect()
{
 double rho = setpointEdit1->text().toDouble();
 double gama = setpointEdit2->text().toDouble();
 double psi = setpointEdit3->text().toDouble();
 QVector<double> setpoint = {rho, gama, psi};
 emit setpointChanged(setpoint);
}
