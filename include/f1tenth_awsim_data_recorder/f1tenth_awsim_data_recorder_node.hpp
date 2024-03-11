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

#ifndef F1TENTH_AWSIM_DATA_RECORDER__F1TENTH_AWSIM_DATA_RECORDER_NODE_HPP_
#define F1TENTH_AWSIM_DATA_RECORDER__F1TENTH_AWSIM_DATA_RECORDER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "f1tenth_awsim_data_recorder/f1tenth_awsim_data_recorder.hpp"

namespace f1tenth_awsim_data_recorder
{
using F1tenthAwsimDataRecorderPtr = std::unique_ptr<f1tenth_awsim_data_recorder::F1tenthAwsimDataRecorder>;

class F1TENTH_AWSIM_DATA_RECORDER_PUBLIC F1tenthAwsimDataRecorderNode : public rclcpp::Node
{
public:
  explicit F1tenthAwsimDataRecorderNode(const rclcpp::NodeOptions & options);

private:
  F1tenthAwsimDataRecorderPtr f1tenth_awsim_data_recorder_{nullptr};
  int64_t param_name_{123};
};
}  // namespace f1tenth_awsim_data_recorder

#endif  // F1TENTH_AWSIM_DATA_RECORDER__F1TENTH_AWSIM_DATA_RECORDER_NODE_HPP_
