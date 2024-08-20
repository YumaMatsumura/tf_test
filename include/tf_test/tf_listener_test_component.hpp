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

#ifndef TF_TEST__TF_LISTENER_TEST_COMPONENT_HPP_
#define TF_TEST__TF_LISTENER_TEST_COMPONENT_HPP_

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace tf_test
{

class TfListenerTest : public rclcpp::Node
{
public:
  explicit TfListenerTest(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TfListenerTest();

private:
  void timerCallback();

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // tf_test

#endif  // TF_TEST__TF_LISTENER_TEST_COMPONENT_HPP_
