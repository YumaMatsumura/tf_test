// Copyright 2024 Yuma Matsumura All rights reserved.
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

#include "tf_test/tf_listener_test_component.hpp"

namespace tf_test
{

TfListenerTest::TfListenerTest(const rclcpp::NodeOptions & options)
: Node("tf_listener_test_node", options)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  timer_ = this->create_wall_timer(
    std::chrono::microseconds(1000), std::bind(&TfListenerTest::timerCallback, this));
}

TfListenerTest::~TfListenerTest()
{
}

void TfListenerTest::timerCallback()
{
  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped =
      tf_buffer_->lookupTransform("base_link", "lidar_link", tf2::TimePointZero);
    RCLCPP_INFO(
      this->get_logger(), 
      "Transform from base_link to lidar_link:\n"
      "  Translation - x: %f, y: %f, z: %f\n"
      "  Rotation    - x: %f, y: %f, z: %f, w: %f",
      transform_stamped.transform.translation.x,
      transform_stamped.transform.translation.y,
      transform_stamped.transform.translation.z,
      transform_stamped.transform.rotation.x,
      transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z,
      transform_stamped.transform.rotation.w);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform from base_link to lidar_link: %s", ex.what());
  }
}

}  // namespace tf_test

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tf_test::TfListenerTest)
