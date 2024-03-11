// Copyright 2024 Szymon
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "f1tenth_awsim_data_recorder/f1tenth_awsim_data_recorder_node.hpp"

namespace f1tenth_awsim_data_recorder
{

F1tenthAwsimDataRecorderNode::F1tenthAwsimDataRecorderNode(const rclcpp::NodeOptions & options)
:  Node("f1tenth_awsim_data_recorder", options)
{
  f1tenth_awsim_data_recorder_ = std::make_unique<f1tenth_awsim_data_recorder::F1tenthAwsimDataRecorder>();
  param_name_ = this->declare_parameter("param_name", 456);
  f1tenth_awsim_data_recorder_->foo(param_name_);
}

}  // namespace f1tenth_awsim_data_recorder

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(f1tenth_awsim_data_recorder::F1tenthAwsimDataRecorderNode)
