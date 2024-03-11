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

#ifndef F1TENTH_AWSIM_DATA_RECORDER__F1TENTH_AWSIM_DATA_RECORDER_HPP_
#define F1TENTH_AWSIM_DATA_RECORDER__F1TENTH_AWSIM_DATA_RECORDER_HPP_

#include <cstdint>

#include "f1tenth_awsim_data_recorder/visibility_control.hpp"


namespace f1tenth_awsim_data_recorder
{

class F1TENTH_AWSIM_DATA_RECORDER_PUBLIC F1tenthAwsimDataRecorder
{
public:
  F1tenthAwsimDataRecorder();
  int64_t foo(int64_t bar) const;
};

}  // namespace f1tenth_awsim_data_recorder

#endif  // F1TENTH_AWSIM_DATA_RECORDER__F1TENTH_AWSIM_DATA_RECORDER_HPP_
