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

#ifndef CORE_WINDOW_H
#define CORE_WINDOW_H

#include "dpgui_global.h"
#include "polar_plot.h"
#include "trajectory.h"
#include <QTimer>
#include <QVector>
#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

class PolarPlot;
class Trajectory;

class DPGUISHARED_EXPORT CoreWindow : public QWidget {
  Q_OBJECT
public:
  explicit CoreWindow(QWidget *parent = nullptr);

  double getScaleRatio() const { return polarPlot->getScaleRatio(); }

signals:
  void poseChanged(const QVector<double> &);
  void setpointChanged(const QVector<double> &);
  void servo_close();

private:
  PolarPlot *polarPlot;
  Trajectory *trajectory;
  QLabel *setpointLable = new QLabel("定位点");
  QLineEdit *setpointEdit1 = new QLineEdit("10");
  QLineEdit *setpointEdit2 = new QLineEdit("0");
  QLineEdit *setpointEdit3 = new QLineEdit("0");
  QPushButton *confirmButton = new QPushButton("确定");
  QPushButton *closeButton = new QPushButton("关闭");
  QLabel *kpLabel = new QLabel("比例系数");
  QLineEdit *kpEdit1 = new QLineEdit("1");
  QLineEdit *kpEdit2 = new QLineEdit("0");
  QLineEdit *kpEdit3 = new QLineEdit("0");
  QLabel *kdLabel = new QLabel("微分系数");
  QLineEdit *kdEdit1 = new QLineEdit("1");
  QLineEdit *kdEdit2 = new QLineEdit("1");
  QLineEdit *kdEdit3 = new QLineEdit("100");

public slots:
  void messageReceived(const QVector<double> &msg);
  void setpointSelect();
};

#endif // CORE_WINDOW_H
