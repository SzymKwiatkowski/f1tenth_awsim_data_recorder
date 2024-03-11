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

#include "rclcpp/qos.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace f1tenth_awsim_data_recorder
{

F1tenthAwsimDataRecorderNode::F1tenthAwsimDataRecorderNode(const rclcpp::NodeOptions & options)
:  Node("f1tenth_awsim_data_recorder", options)
{
  rclcpp::QoS qos = rclcpp::QoS(10);
  f1tenth_awsim_data_recorder_ = std::make_unique<f1tenth_awsim_data_recorder::F1tenthAwsimDataRecorder>();

  std::string ackermann_topic = declare_parameter("ackermann_topic", "/control/command/control_cmd");
  std::string ground_truth_topic = declare_parameter("ground_truth_topic", "/awsim/ground_truth/vehicle/pose");
  std::string trajectory_topic = declare_parameter("trajectory_topic", "/control/trajectory_follower/lateral/predicted_trajectory");
  ackerman_sub_.subscribe(this, ackermann_topic, qos.get_rmw_qos_profile());
  ground_truth_topic_sub_.subscribe(this, ground_truth_topic, qos.get_rmw_qos_profile());
  trajectory_sub_.subscribe(this, trajectory_topic, qos.get_rmw_qos_profile());

  _synchronizer = std::make_shared<message_filters::TimeSynchronizer<
  autoware_auto_control_msgs::msg::AckermannControlCommand,
  geometry_msgs::msg::PoseStamped,
  autoware_auto_planning_msgs::msg::Trajectory>>(
    ackerman_sub_,
    ground_truth_topic_sub_,
    trajectory_sub_,
    10
  );

  _synchronizer->registerCallback(
    std::bind(&F1tenthAwsimDataRecorderNode::SyncCallback, this, _1, _2, _3)
  );
}


void F1tenthAwsimDataRecorderNode::SyncCallback(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr & ackermann,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & ground_truth,
    const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory
  )
{
  RCLCPP_INFO(this->get_logger(), "Sync callback with %u, and %u as times", ackermann->stamp, ground_truth->header.stamp.sec);
  f1tenth_awsim_data_recorder_->SaveToCsv(ackermann, ground_truth, trajectory);
}

}  // namespace f1tenth_awsim_data_recorder

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(f1tenth_awsim_data_recorder::F1tenthAwsimDataRecorderNode)
