// Copyright 2024 Szymon Kwiatkowski
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

#ifndef F1TENTH_AWSIM_DATA_RECORDER__F1TENTH_AWSIM_DATA_RECORDER_HPP_
#define F1TENTH_AWSIM_DATA_RECORDER__F1TENTH_AWSIM_DATA_RECORDER_HPP_

#include <cstdint>
#include <fstream>
#include <string>
#include <stdexcept>

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "f1tenth_awsim_data_recorder/visibility_control.hpp"

namespace f1tenth_awsim_data_recorder
{

class F1TENTH_AWSIM_DATA_RECORDER_PUBLIC F1tenthAwsimDataRecorder
{
public:
  F1tenthAwsimDataRecorder();
  ~F1tenthAwsimDataRecorder();

  void SetMaxPoints(size_t max_points_count);

  void SaveToCsv(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & position,
    const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory);

  void SaveToCsv(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr & ackermann,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & position,
    const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory);

private:
  size_t _max_point_count = 100;
  std::ofstream _file;
  template<typename ... Args>
  std::string DynamicConversion(const std::string& format, Args ... args);

  std::string ConvertAckermannAndPose(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr & ackermann,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & position
  );
  std::string ZerosForPoint();
  std::string ConvertPoints(autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_point);
  std::string HeaderToCsv();
};

}  // namespace f1tenth_awsim_data_recorder

#endif  // F1TENTH_AWSIM_DATA_RECORDER__F1TENTH_AWSIM_DATA_RECORDER_HPP_
