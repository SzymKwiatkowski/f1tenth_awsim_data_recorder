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

#include "f1tenth_awsim_data_recorder/f1tenth_awsim_data_recorder.hpp"

#include <iostream>

namespace f1tenth_awsim_data_recorder
{

F1tenthAwsimDataRecorder::F1tenthAwsimDataRecorder()
{
    // Open file
    _file.open ("data.csv");
}

void F1tenthAwsimDataRecorder::SetMaxPoints(size_t max_points_count)
{
    _max_point_count = max_points_count;

    // write columns labels
    std::string header = HeaderToCsv();

    _file << header + "\n";
}

F1tenthAwsimDataRecorder::~F1tenthAwsimDataRecorder()
{
    _file.close();
}

void F1tenthAwsimDataRecorder::SaveToCsv(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & ground_truth,
    const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory)
{
}

void F1tenthAwsimDataRecorder::SaveToCsv(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr & ackermann,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & position,
    const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory)
{
    std::string result_to_write = ConvertAckermannAndPose(ackermann, position);

    for (size_t i=0; i < _max_point_count; i++)
    {
        if (i < trajectory->points.size())
        {
            std::string pointConversion = ConvertPoints(trajectory->points[i]);

            result_to_write+=pointConversion;
        }
        else
        {
            result_to_write+=ZerosForPoint();
        }
    }

    _file << result_to_write + " \n"; // append endline to start from new row
}

std::string F1tenthAwsimDataRecorder::ConvertAckermannAndPose(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr &ackermann,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr &position)
{
    std::string ackermann_pose_converted = DynamicConversion(
        "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
        ackermann->lateral.steering_tire_angle,
        ackermann->lateral.steering_tire_rotation_rate,
        ackermann->longitudinal.acceleration,
        ackermann->longitudinal.speed,
        ackermann->longitudinal.jerk,
        position->pose.position.x,
        position->pose.position.y,
        position->pose.position.z,
        position->pose.orientation.x,
        position->pose.orientation.y,
        position->pose.orientation.z,
        position->pose.orientation.w);
    
    return ackermann_pose_converted;
}

std::string F1tenthAwsimDataRecorder::ZerosForPoint()
{
    return DynamicConversion(
                ", %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            );
}

std::string F1tenthAwsimDataRecorder::ConvertPoints(autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_point)
{
    std::string pointConversion = DynamicConversion(
                ", %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
                trajectory_point.acceleration_mps2,
                trajectory_point.front_wheel_angle_rad,
                trajectory_point.heading_rate_rps,
                trajectory_point.lateral_velocity_mps,
                trajectory_point.longitudinal_velocity_mps,
                trajectory_point.rear_wheel_angle_rad,
                trajectory_point.pose.position.x,
                trajectory_point.pose.position.y,
                trajectory_point.pose.position.z,
                trajectory_point.pose.orientation.x,
                trajectory_point.pose.orientation.y,
                trajectory_point.pose.orientation.z,
                trajectory_point.pose.orientation.w
            );

    return pointConversion;
}

std::string F1tenthAwsimDataRecorder::HeaderToCsv()
{
    std::string base_header = "steering_tire_angle, steering_tire_rotation_rate, acceleration, speed, jerk, pose_x, pose_y, pose_z, orientation_x, orientation_y, orientation_z, orientation_w";

    for (size_t i=0; i < _max_point_count; i++)
    {
        base_header += DynamicConversion(
                ", point_%i_acceleration_mps2, \
                point_%i_front_wheel_angle_rad, \
                point_%i_heading_rate_rps, \
                point_%i_lateral_velocity_mps, \
                point_%i_longitudinal_velocity_mps, \
                point_%i_rear_wheel_angle_rad, \
                point_%i_pos_x, \
                point_%i_pos_y, \
                point_%i_pos_z, \
                point_%i_orientation_x, \
                point_%i_orientation_y, \
                point_%i_orientation_z, \
                point_%i_orientation_w",
                i,
                i,
                i,
                i,
                i,
                i,
                i,
                i,
                i,
                i,
                i,
                i,
                i,
                i
            );
    }

    return base_header;
}

template<typename ... Args>
std::string F1tenthAwsimDataRecorder::DynamicConversion(const std::string& format, Args ... args)
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}
}  // namespace f1tenth_awsim_data_recorder
