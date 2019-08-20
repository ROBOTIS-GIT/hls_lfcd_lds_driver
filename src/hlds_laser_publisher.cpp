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

 /* Authors: Pyo, Darby Lim, SP Kong, JH Yang */
 /* maintainer: Pyo */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <boost/asio.hpp>
#include <hls_lfcd_lds_driver/lfcd_laser.hpp>

namespace hls_lfcd_lds
{
LFCDLaser::LFCDLaser(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
  : port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
{
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

  boost::asio::write(serial_, boost::asio::buffer("b", 1));  // start motor
}

LFCDLaser::~LFCDLaser()
{
  boost::asio::write(serial_, boost::asio::buffer("e", 1));  // stop motor
}

void LFCDLaser::poll(sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  uint8_t start_count = 0;
  bool got_scan = false;
  boost::array<uint8_t, 2520> raw_bytes;
  uint8_t good_sets = 0;
  uint32_t motor_speed = 0;
  rpms=0;
  int index;

  while (!shutting_down_ && !got_scan)
  {
    // Wait until first data sync of frame: 0xFA, 0xA0
    boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count],1));

    if(start_count == 0)
    {
      if(raw_bytes[start_count] == 0xFA)
      {
        start_count = 1;
      }
    }
    else if(start_count == 1)
    {
      if(raw_bytes[start_count] == 0xA0)
      {
        start_count = 0;

        // Now that entire start sequence has been found, read in the rest of the message
        got_scan = true;

        boost::asio::read(serial_,boost::asio::buffer(&raw_bytes[2], 2518));

        scan->angle_increment = (2.0*M_PI/360.0);
        scan->angle_min = 0.0;
        scan->angle_max = 2.0*M_PI-scan->angle_increment;
        scan->range_min = 0.12;
        scan->range_max = 3.5;
        scan->ranges.resize(360);
        scan->intensities.resize(360);

        //read data in sets of 6
        for(uint16_t i = 0; i < raw_bytes.size(); i=i+42)
        {
          if(raw_bytes[i] == 0xFA && raw_bytes[i+1] == (0xA0 + i / 42)) //&& CRC check
          {
            good_sets++;
            motor_speed += (raw_bytes[i+3] << 8) + raw_bytes[i+2]; //accumulate count for avg. time increment
            rpms=(raw_bytes[i+3]<<8|raw_bytes[i+2])/10;

            for(uint16_t j = i+4; j < i+40; j=j+6)
            {
              index = 6*(i/42) + (j-4-i)/6;

              // Four bytes per reading
              uint8_t byte0 = raw_bytes[j];
              uint8_t byte1 = raw_bytes[j+1];
              uint8_t byte2 = raw_bytes[j+2];
              uint8_t byte3 = raw_bytes[j+3];

              uint16_t intensity = (byte1 << 8) + byte0;
              uint16_t range = (byte3 << 8) + byte2;

              scan->ranges[359-index] = range / 1000.0;
              scan->intensities[359-index] = intensity;
            }
          }
        }

        scan->time_increment = motor_speed/good_sets/1e8;
      }
      else
      {
        start_count = 0;
      }
    }
  }
}
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("hlds_laser_publisher");
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
  boost::asio::io_service io;

  std::string port;
  std::string frame_id;
  int baud_rate;

  node->declare_parameter("port");
  node->declare_parameter("frame_id");

  node->get_parameter_or<std::string>("port", port, "/dev/ttyUSB0");
  node->get_parameter_or<std::string>("frame_id", frame_id, "laser");

  baud_rate = 230400;

  RCLCPP_INFO(node->get_logger(), "Init hlds_laser_publisher Node Main");
  RCLCPP_INFO(node->get_logger(), "port : %s frame_id : %s", port.c_str(), frame_id.c_str());

  try
  {
    hls_lfcd_lds::LFCDLaser laser(port, baud_rate, io);
    laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::SensorDataQoS()));

    while (rclcpp::ok())
    {
      auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
      scan->header.frame_id = frame_id;
      laser.poll(scan);
      scan->header.stamp = node->now();
      laser_pub->publish(*scan);
    }
    laser.close();

    return 0;
  }
  catch (boost::system::system_error ex)
  {
    //ROS_ERROR("An exception was thrown: %s", ex.what());
    return -1;
  }
}
