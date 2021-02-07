/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Bo Li <lither@sjtu.edu.cn>
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

#include "controller.h"
#include <QDebug>

Controller::Controller(const QString &mode, QObject *parent)
    : QObject(parent), positioning_mode(mode) {

  qDebug() << "controller has started";
  // define the controller interface
  VesselControlProperty vessel;
  vessel.name = "Oil_rig";
  vessel.mass = 102;
  vessel.Izz = 82.62;
  vessel.a11 = 51.0;
  vessel.a22 = 102.0;
  vessel.a33 = 82.62;
  vessel.xg = 0;
  vessel.d11 = 102.0 / 300;
  vessel.d22 = 10.2;
  vessel.d33 = 8.62;

  ThrustAllocationConfiguration config;
  config.azimuth_thruster_number = 8;
  config.tunnel_thruster_number = 0;
  config.main_thruster_number = 0;
  //螺旋桨位置 需要改
  config.x_coordinates = {15.70,  47.02,  47.02,  15.70,
                          -15.70, -47.02, -47.02, -15.7};
  config.y_coordinates = {35.50,  24.58,  -24.58, -35.50,
                          -35.50, -24.58, 24.58,  35.50};
  config.thrust_upper_limit = {800, 800, 800, 800,
                               800, 800, 800, 800}; // unit: kN
  config.thrust_lower_limit = {10, 10, 10, 10, 10, 10, 10, 10};
  config.thrust_rate_limit = {20, 20, 20, 20, 20, 20, 20, 20};
  config.rotation_rate_limit = {10, 10, 10, 10, 10, 10, 10, 10}; // unit: deg/s
  config.weights = {1, 1E2, 1E10};
  config.initial_azimuth = {2, 2, -2, -2, -2, -2, 2, 2};
  config.initial_thrust = {30, 40, 20, 30, 20, 12, 19, 30};

  qDebug() << "controller 实例化 controller interface";
  controller_interface = new ControllerInterface(
      vessel, config, {{10.0, 0.0, 0.0}}, 0.1, "C:/tugboat/semi/results/");
  qDebug() << "controller 实例化 controller interface 成功";
  controller_interface->moveToThread(&controller_thread);

  connect(&controller_thread, &QThread::finished, controller_interface,
          &QObject::deleteLater);
  connect(this, &Controller::messageReceived, controller_interface,
          &ControllerInterface::messageReceived);
  connect(controller_interface, &ControllerInterface::messageReady, this,
          &Controller::messageReady);
  connect(controller_interface, &ControllerInterface::thrusterReady, this,
          &Controller::thrusterReady);
  connect(this, &Controller::setpointReceived, controller_interface,
          &ControllerInterface::setpointReceived);

  controller_thread.start();
  qDebug() << "controller has started done";
}

Controller::~Controller() {
  controller_thread.quit();
  controller_thread.wait();
  qDebug() << "controller has exited";
}
