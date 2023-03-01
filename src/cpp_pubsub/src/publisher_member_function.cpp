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
#include <unistd.h>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include "sensor_msgs/msg/array256.hpp"
#include "sensor_msgs/msg/array512.hpp"
#include "sensor_msgs/msg/array1k.hpp"
#include "sensor_msgs/msg/array4k.hpp"
#include "sensor_msgs/msg/array8k.hpp"
#include "sensor_msgs/msg/array16k.hpp"
#include "sensor_msgs/msg/array32k.hpp"
#include "sensor_msgs/msg/array64k.hpp"
#include "sensor_msgs/msg/array128k.hpp"
#include "sensor_msgs/msg/array256k.hpp"
//#include "sensor_msgs/msg/point_cloud1m.hpp"
//#include "sensor_msgs/msg/point_cloud2m.hpp"
//#include "sensor_msgs/msg/point_cloud4m.hpp"
//#include "sensor_msgs/msg/point_cloud512k.hpp"
//#include "sensor_msgs/msg/point_cloud8m.hpp"
#include <sys/time.h>
#include <functional>

using namespace std::chrono_literals;
using std::chrono::nanoseconds;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
std::ofstream writing_file;
// std::string filename = "pub.txt";
// writing_file.open("pub.txt", std::ios::app);

size_t count_;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher")//, count_(0)
  {
    writing_file.open("pub.txt", std::ios::app);
    publisher_ = this->create_publisher<sensor_msgs::msg::Array256k>("topic",rclcpp::QoS(10000).reliable());
    // for(int count=0;count<10030;count++)
    // {
    // callback();
    // }

  }
  void callback()
  {
  RCLCPP_INFO(
      this->get_logger(), "Publishing PointCloud2: %ld ", count_);
  auto msg_pc_ = sensor_msgs::msg::Array256k();
  auto time_now=this->now();
  msg_pc_.header.stamp = time_now;
  msg_pc_.header.frame_id = std::to_string(count_);

  auto now = std::chrono::system_clock::now();

  publisher_->publish(msg_pc_);
  
  if (count_>=30)
  {
    // auto end = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch);
    long now1 = value.count();
    writing_file << now1 << std::endl;
  }
  count_++;
  usleep(10000);
  }


private:
  rclcpp::Publisher<sensor_msgs::msg::Array256k>::SharedPtr publisher_;
  // size_t count_=0;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  MinimalPublisher node;
  // rclcpp::spin(std::make_shared<MinimalPublisher>());
  // // callback
  for(int i=0;i<10030;i++)
  { 
    node.callback();
  }
  writing_file.close();
  rclcpp::shutdown();
  return 0;
}
