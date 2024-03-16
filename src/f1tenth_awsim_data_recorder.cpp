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
    _file.open ("example.csv");


    // write columns labels
    _file << "";
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
    // _file << "This is the first cell in the first column.\n";
    // _file << "a,b,c,\n";
    // _file << "c,s,v,\n";
    // _file << "1,2,3.456\n";
    // _file << "semi;colon";

    std::string result_to_write = DynamicConversion(
        "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %u \n",
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
        position->pose.orientation.w,
        trajectory->points.max_size());

    _file << result_to_write;
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
