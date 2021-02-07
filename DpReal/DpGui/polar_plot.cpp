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

#include "polar_plot.h"
#include <QPainter>
#include <QPen>

PolarPlot::PolarPlot(QWidget *parent) : QWidget(parent) {
  //    resize(1000, 1000);
}

void PolarPlot::paintEvent(QPaintEvent * /*event*/) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  int side = qMin(width(), height());
  painter.setViewport((width() - side) / 2, (height() - side) / 2, side, side);
  painter.setWindow(-110, -110, 220, 220);
  drawCoordinate(painter);
}

void PolarPlot::drawCoordinate(QPainter &painter) {
  QPen thickPen(palette().windowText(), 0.3);

  // draw the big circle
  qreal circleRadius = 100;
  painter.setPen(thickPen);
  QRectF rectangle(-circleRadius, -circleRadius, 2 * circleRadius,
                   2 * circleRadius);
  painter.drawEllipse(rectangle);

  QFont font;
  font.setPointSize(2);
  font.setFamily("Arial");

  // draw the ticks
  for (int i = 0; i <= 35; ++i) {
    painter.save();
    painter.rotate(i * 10);
    painter.drawLine(0, -circleRadius, 0, -circleRadius - 1);
    painter.setFont(font);

    QRectF textRect(-3, -105, 6, 4);
    if (i == 0)
      painter.drawText(textRect, Qt::AlignHCenter | Qt::AlignVCenter, "N");
    else if (i == 9)
      painter.drawText(textRect, Qt::AlignHCenter | Qt::AlignVCenter, "E");
    else if (i == 18)
      painter.drawText(textRect, Qt::AlignHCenter | Qt::AlignVCenter, "S");
    else if (i == 27)
      painter.drawText(textRect, Qt::AlignHCenter | Qt::AlignVCenter, "W");
    else
      painter.drawText(textRect, Qt::AlignHCenter | Qt::AlignVCenter,
                       QString::number(10 * i));

    painter.restore();
  }

  // draw the inner circles
  int circleNum = int(maxDistance / gridDistance);
  QPen thinPen;
  thinPen.setStyle(Qt::DashLine);
  thinPen.setWidthF(0.1);

  for (int i = 1; i < circleNum; ++i) {
    double radius = circleRadius * i * gridDistance / maxDistance;
    QRectF rectangle(-radius, -radius, 2 * radius, 2 * radius);

    if (i == int(circleNum / 2))
      painter.setPen(thickPen);
    else
      painter.setPen(thinPen);

    painter.drawEllipse(rectangle);
  }

  // draw cross lines
  painter.drawLine(-circleRadius, 0, circleRadius, 0);
  painter.drawLine(0, -circleRadius, 0, circleRadius);
}
