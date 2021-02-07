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

#include "trajectory.h"
#include <QPainter>
#include <QTimer>
#include <QtMath>

Trajectory::Trajectory(QWidget *parent) : QWidget(parent) {
  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(update()));
  timer->start(100);

  centerList.append(QPointF(0, 0));
}

void Trajectory::drawTrajectory(QPainter &painter,
                                const QVector<QPointF> &list) {
  for (auto point = list.cbegin(); point != list.cend() - 1; ++point) {
    painter.drawLine(*point, *(point + 1));
  }
}

// definition of the heading ???? modified
void Trajectory::drawVessel(QPainter &painter, const double &x_coor,
                            const double &y_coor, const double &heading) {
  painter.save();
  painter.rotate(-heading);
  painter.setBrush(Qt::blue);

  // draw the direction indicator
  static const QPointF points_dir[3] = {QPointF(-2, -96), QPointF(2, -96),
                                        QPointF(0, -99)};
  painter.drawConvexPolygon(points_dir, 3);

  painter.rotate(heading);
  painter.translate(x_coor, y_coor);
  painter.rotate(-heading);

  // draw COG
  painter.drawEllipse(-1, -1, 2, 2);

  // draw the vessel
  static const QPointF points[9] = {
      QPointF(-15, 20), QPointF(-15, -20), QPointF(-10, -25),
      QPointF(-5, -20), QPointF(5, -20),   QPointF(10, -25),
      QPointF(15, -20), QPointF(15, 20),   QPointF(-15, 20)};

  painter.setBrush(Qt::NoBrush);
  painter.drawConvexPolygon(points, 9);

  painter.restore();
}

void Trajectory::paintEvent(QPaintEvent * /*event*/) {
  QPainter painter(this);
  QPen trajectoryPen(QColor(Qt::magenta), 0.4);
  QPen vesselPen(QColor(Qt::blue), 0.8);
  vesselPen.setJoinStyle(Qt::RoundJoin);

  painter.setRenderHint(QPainter::Antialiasing);

  int side = qMin(width(), height());
  painter.setViewport((width() - side) / 2, (height() - side) / 2, side, side);
  painter.setWindow(-110, -110, 220, 220);

  painter.setPen(trajectoryPen);
  drawTrajectory(painter, centerList);

  painter.setPen(vesselPen);
  drawVessel(painter, centerList.back().rx(), centerList.back().ry(),
             vesselHeading);
}

void Trajectory::updatePosition(const QVector<double> &pose) {
  QPointF center;
  center.setX(-pose.at(1));
  center.setY(-pose.at(0));
  centerList.append(center);

  // limit the trajectory length
  if (centerList.size() > 6000)
    centerList.pop_front();

  vesselHeading = pose.at(2);
}
