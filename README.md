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

