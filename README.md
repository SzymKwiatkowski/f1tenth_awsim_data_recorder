# f1tenth_awsim_data_recorder
<!-- Required -->
<!-- Package description -->

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-up-to f1tenth_awsim_data_recorder
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

```bash
ros2 launch f1tenth_awsim_data_recorder f1tenth_awsim_data_recorder.launch.py
```

### Parameters
Used via `f1tenth_awsim_data_recorder.param.yaml` file in config directory.
| Name         | Type | Description  |
| ------------ | ---- | ------------ |
| max_points_count | int  | Maximum count of points saved (remaining if path is shorter then specified amount it is filled with 0) |
| ackermann_topic | string | Sample desc. |
| ground_truth_topic | string | Sample desc. |
| trajectory_topic | string | Sample desc. |


# Recording data 
Simple step by step process of data preparation for recording. Recorder can be used on prerecorder bag data as well as live simulation. Keep in mind that recorder has `autoware_auto_msgs` as dependencies.

## Bag recording method
Launch simulator in autoware docker:
```bash
./autoware_awsim/AWSIM_v1.2.0_F1TENTH/AWS_v1.2.0.x86_64 
```
Launch simulator:
```bash
ros2 launch f1tenth_launch e2e_simulator.launch.py 
```
Launch bag recording in paused mode:
```bash
ros2 bag record -o recording_name --start-paused /control/command/control_cmd /awsim/ground_truth/vehicle/pose /localization/cartographer/pose /planning/racing_planner/trajectory
```
In theory only:
- `/control/command/control_cmd`
- `/localization/cartographer/pose`
- `/planning/racing_planner/trajectory`
  
topic are needed.

Then launch simulator in autonomous drive mode and start/stop bag recording via pressing `space`. If PC is not able to keep up slow down simulation.

## Running recorder
To run recorder adjust parameters in config file (example data):
```yaml
/**:
  ros__parameters:
    max_points_count: 271 # Max points count being saved from trajectory topic
    ackermann_topic: "/control/command/control_cmd"
    ground_truth_topic: "/localization/cartographer/pose"
    trajectory_topic:  "/planning/racing_planner/trajectory"
```
After that build project and then just run it via `ros2 launch` command specified in `Usage` section.