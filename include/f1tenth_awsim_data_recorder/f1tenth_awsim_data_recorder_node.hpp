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
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/latest_time.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "f1tenth_awsim_data_recorder/f1tenth_awsim_data_recorder.hpp"

namespace f1tenth_awsim_data_recorder
{
using F1tenthAwsimDataRecorderPtr = std::unique_ptr<f1tenth_awsim_data_recorder::F1tenthAwsimDataRecorder>;

typedef message_filters::sync_policies::LatestTime<
autoware_auto_control_msgs::msg::AckermannControlCommand,
geometry_msgs::msg::PoseStamped,
autoware_auto_planning_msgs::msg::Trajectory> lastest_policy;

typedef message_filters::Synchronizer<lastest_policy> LatestSynchronizer;

typedef message_filters::sync_policies::ApproximateTime<
geometry_msgs::msg::PoseStamped,
autoware_auto_planning_msgs::msg::Trajectory> approximate_policy;

typedef message_filters::Synchronizer<approximate_policy> ApproximateSynchronizer;

class F1TENTH_AWSIM_DATA_RECORDER_PUBLIC F1tenthAwsimDataRecorderNode : public rclcpp::Node
{
public:
  explicit F1tenthAwsimDataRecorderNode(const rclcpp::NodeOptions & options);

private:
  F1tenthAwsimDataRecorderPtr f1tenth_awsim_data_recorder_{nullptr};
  
  std::shared_ptr<message_filters::TimeSynchronizer<
  geometry_msgs::msg::PoseStamped,
  autoware_auto_planning_msgs::msg::Trajectory>> _time_synchronizer;

  std::shared_ptr<LatestSynchronizer> _synchronizer;

  message_filters::Subscriber<autoware_auto_control_msgs::msg::AckermannControlCommand> ackerman_sub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> ground_truth_topic_sub_;
  message_filters::Subscriber<autoware_auto_planning_msgs::msg::Trajectory> trajectory_sub_;

  void SyncCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & ground_truth,
    const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory
  );

  void SynchronizerCallback(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr & ackermann,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & ground_truth,
    const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory
  );

  void AckermannCallback(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr & ackermann
  );
};
}  // namespace f1tenth_awsim_data_recorder

#endif  // F1TENTH_AWSIM_DATA_RECORDER__F1TENTH_AWSIM_DATA_RECORDER_NODE_HPP_
