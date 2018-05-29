/*******************************************************************************
* Copyright (c) 2016, Hitachi-LG Data Storage
* Copyright (c) 2017, ROBOTIS
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Authors: SP Kong, JH Yang, Pyo */
/* maintainer: Pyo */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <hls_lfcd_lds_driver/hlds_laser_segment_publisher.h>

namespace hls_lfcd_lds
{
LFCDLaser::LFCDLaser(boost::asio::io_service& io)
: serial_(io),
  shutting_down_(false)
{
  nh_.param("port", port_, std::string("/dev/ttyUSB0"));
  nh_.param("baud_rate", baud_rate_, 230400);
  nh_.param("frame_id", frame_id_, std::string("base_scan"));
  nh_.param("lfcd_start", lfcdstart_, 1);
  nh_.param("lfcd_stop", lfcdstop_, 2);
  nh_.param("lfcd_start", lfcdstartstop_, lfcdstart_);

  serial_.open(port_);
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

  if(lfcdstartstop_ == 1)
  {
    // Below command is not required after firmware upgrade (2017.10)
    boost::asio::write(serial_, boost::asio::buffer("b", 1));
  }
  else if(lfcdstartstop_ == 2)
  {
    boost::asio::write(serial_, boost::asio::buffer("e", 1));
  }

  scan_.header.frame_id = frame_id_;
  scan_.angle_increment = (2.0*M_PI/360.0);
  scan_.angle_min = 0.0;
  scan_.angle_max = 2.0*M_PI-scan_.angle_increment;
  scan_.range_min = 0.12;
  scan_.range_max = 3.5;
  scan_.ranges.resize(360);
  scan_.intensities.resize(360);

  laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 100);
}

void LFCDLaser::poll()
{
  bool got_scan = false;
  boost::array<uint8_t, 42> raw_bytes;
  uint32_t motor_speed = 0;
  int index;


  while (!shutting_down_ && !got_scan)
  {
    boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[0],1));

    if(raw_bytes[0] == 0xFA)
    {
      got_scan = true;
      boost::asio::read(serial_,boost::asio::buffer(&raw_bytes[1], 41));

      if(raw_bytes[1] >= 0xA0  && raw_bytes[1] <= 0xDB) // TODO: checksum
      {
        int degree_count_num = 0;

        index = (raw_bytes[1] - 0xA0) * 6;

        for(uint16_t j = 4; j < 40; j = j + 6)
        {
          uint8_t byte0 = raw_bytes[j];
          uint8_t byte1 = raw_bytes[j+1];
          uint8_t byte2 = raw_bytes[j+2];
          uint8_t byte3 = raw_bytes[j+3];

          uint16_t intensity = (byte1 << 8) + byte0;
          uint16_t range     = (byte3 << 8) + byte2;

          scan_.ranges[359 - index - degree_count_num] = range / 1000.0;
          scan_.intensities[359 - index - degree_count_num] = intensity;

          degree_count_num++;
        }

        scan_.time_increment = 0.2 / 360; // 1sec / 5scan/ 360beam
        scan_.scan_time = 0.2 / 60;
        scan_.header.stamp = ros::Time::now();
        laser_pub_.publish(scan_);
        ros::spinOnce();
      }
    }
  }
}

void LFCDLaser::close()
{
  shutting_down_ = true;
  serial_.open(port_);
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  boost::asio::write(serial_, boost::asio::buffer("e", 1));
};
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "hlds_laser_segment_publisher");

  boost::asio::io_service io;
  hls_lfcd_lds::LFCDLaser laser(io);

  while (ros::ok())
  {
    laser.poll();
  }
  laser.close();

  return 0;
}
