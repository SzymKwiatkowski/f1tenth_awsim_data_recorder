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

#include "f1tenth_awsim_data_recorder/f1tenth_awsim_data_recorder.hpp"

#include <iostream>

namespace f1tenth_awsim_data_recorder
{

F1tenthAwsimDataRecorder::F1tenthAwsimDataRecorder()
{
}

void F1tenthAwsimDataRecorder::SaveToCsv(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & ground_truth,
    const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory)
{
}

}  // namespace f1tenth_awsim_data_recorder
