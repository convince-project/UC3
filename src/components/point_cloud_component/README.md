# Point Cloud Component

Subscribes to a PointCloud2 stream and outputs a boolean "is_safe" flag (ROS 2 and YARP) based on a Z threshold.

## What it does

- Input: sensor_msgs/PointCloud2 with fields x, y, z (and optionally rgb)
- Logic: counts points where z < z_min_threshold; if violations >= min_violation_count, then is_safe=false
- Output:
  - ROS 2 topic: `/CameraSafety/is_safe` (std_msgs/Bool)
  - YARP port: `.../CameraSafety/is_safe:o` (Bottle: `is_safe <0|1> violations <N>`)

## Parameters

- `input_cloud_topic` (string, default `/camera/depth/color/points`) — Topic to subscribe for the point cloud
- `z_min_threshold` (double, default `0.20`) — Points with `z` smaller than this (in meters, camera frame) are treated as **violations**
- `min_violation_count` (int, default `10`) — Number of violations required before the frame is considered unsafe

## Build & Run
In UC3 repository run: 
```bash
colcon build --base-paths src/* --packages-up-to point_cloud_component
source install/setup.bash

ros2 run point_cloud_component point_cloud_component \
  --ros-args -p input_cloud_topic:=/camera/depth/color/points -p z_min_threshold:=0.20 -p min_violation_count:=10
```

Check output:

```bash
ros2 topic echo /CameraSafety/is_safe
```

YARP:

```bash
yarp read /CameraSafety/is_safe:o
```

## Monitor The Outputs

- **ROS 2**
  ```bash
  ros2 topic echo /CameraSafety/is_safe
  ```
  Example output while the scene is safe:
  ```
  data: true
  ---
  data: true
  ```

- **YARP**
  ```bash
  yarp read /debugSafety:i
  yarp connect /CameraSafety/is_safe:o /debugSafety:i
  ```
  Example output:
  ```
  is_safe 1 violations 0
  is_safe 1 violations 0
  ```

  Each line reports the boolean flag (`0` or `1`) plus how many points currently violate the Z threshold.
