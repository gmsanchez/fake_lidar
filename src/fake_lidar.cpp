// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class FakeLidar : public rclcpp::Node
{

constexpr double deg_2_rad(double x)
{
  return x * M_PI / 180.0;
}

public:
  FakeLidar()
  : Node("fake_lidar"), count_(0)
  {
    laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    frame_id_ = this->declare_parameter("frame_id", std::string("laser_frame"));

    auto timer_callback =
      [this]() -> void {

        auto fake_scan_msg = sensor_msgs::msg::LaserScan();

        fake_scan_msg.header.stamp = this->now();
        fake_scan_msg.header.frame_id = frame_id_;
        fake_scan_msg.angle_min = M_PI - angle_min;
        fake_scan_msg.angle_max = M_PI - angle_max;
        fake_scan_msg.angle_increment = (fake_scan_msg.angle_max - fake_scan_msg.angle_min) / (double)(node_count - 1);

        scan_time = this->now().nanoseconds() * 1E-9;;
        fake_scan_msg.scan_time = scan_time;
        fake_scan_msg.time_increment = scan_time / (double)(node_count - 1);
        fake_scan_msg.range_min = min_distance;
        fake_scan_msg.range_max = max_distance;

        fake_scan_msg.intensities.resize(node_count);
        fake_scan_msg.ranges.resize(node_count);
        for (size_t i = 0; i < node_count; ++i) {
          fake_scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
          fake_scan_msg.intensities[i] = 0;
        }

        this->laser_scan_publisher_->publish(fake_scan_msg);
      };
    timer_ = this->create_wall_timer(200ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
  size_t count_;
  std::string frame_id_;
  double angle_min = deg_2_rad(0);
  double angle_max = deg_2_rad(359);
  size_t node_count = 360 * 8;
  double scan_time;
  const float max_distance = 8.0f;
  const float min_distance = 0.15f;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeLidar>());
  rclcpp::shutdown();
  return 0;
}
