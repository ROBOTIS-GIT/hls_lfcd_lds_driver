/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the Qt Charts module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:GPL$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3 or (at your option) any later version
** approved by the KDE Free Qt Foundation. The licenses are as published by
** the Free Software Foundation and appearing in the file LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef LDSPOLARGRAPH_H
#define LDSPOLARGRAPH_H

#include <QTimer>
#include <QtGui/QMouseEvent>
#include <QtCore/QDebug>
#include <QtCharts/QChartView>
#include <QtCharts/QPolarChart>
#include <QtCharts/QAbstractAxis>
#include <QtCharts/QValueAxis>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QValueAxis>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>

QT_CHARTS_USE_NAMESPACE

class LdsPolarGraph : public QChartView
{
  Q_OBJECT
public:
  LdsPolarGraph(QWidget *parent = 0);
  ~LdsPolarGraph();
  uint16_t* poll();
  void close() { shutting_down_ = true; }

private Q_SLOTS:
  void loadData();

private:
  QTimer timer_;
  QPolarChart *chart_;
  QScatterSeries *series_;
  qreal angular_min_;
  qreal angular_max_;
  qreal radial_min_;
  qreal radial_max;
  boost::asio::io_service io_;
  std::string port_; ///< @brief The serial port the driver is attached to
  uint32_t baud_rate_; ///< @brief The baud rate for the serial connection
  bool shutting_down_; ///< @brief Flag for whether the driver is supposed to be shutting down or not
  boost::asio::serial_port serial_; ///< @brief Actual serial port object for reading/writing to the LFCD Laser Scanner
  uint16_t motor_speed_; ///< @brief current motor speed as reported by the LFCD.
};

#endif
