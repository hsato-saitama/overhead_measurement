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

#include <memory>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
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

#include <chrono>

using std::placeholders::_1;
using std::chrono::nanoseconds;
std::ofstream writing_file;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  { 
    writing_file.open("sub.txt", std::ios::app);

    // auto start = std::chrono::system_clock::now();
    subscription_ = this->create_subscription<sensor_msgs::msg::Array256k>(
      "topic", rclcpp::QoS(10000).reliable(), std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Array256k::UniquePtr msg) const
  {
    auto now = std::chrono::system_clock::now();
    RCLCPP_INFO(
    this->get_logger(), "RelayTimer sub PointCloud2 seq: %s",
    msg->header.frame_id.c_str()); 
    int i = atoi(msg->header.frame_id.c_str());
    if (i>=30)
    {
      // auto end = std::chrono::system_clock::now();
      auto now_ms = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
      auto epoch = now_ms.time_since_epoch();
      auto value = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch);
      long now1 = value.count();
      writing_file << now1 << std::endl;
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::Array256k>::SharedPtr subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // writing_file.open("sub.txt", std::ios::app);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  writing_file.close();
  rclcpp::shutdown();
  return 0;
}
