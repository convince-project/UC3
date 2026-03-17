# simulated_plan_component

ROS 2 Python component that computes `ExecutePoi[]` planning actions from:
- completed POI outcomes (`is_overtime`, `cause`),
- time budget (`current_tour_time`, `max_tour_time`),
- remaining POIs,
- battery percentage.

## Run

```bash
ros2 run simulated_plan_component simulated_plan_component
```

## Help

```bash
ros2 run simulated_plan_component simulated_plan_component --help
```

## ROS Parameters

- `time_per_navigation_poi` (default: `30.0`)
- `time_per_explain_poi` (default: `40.0`)
- `time_per_question_poi` (default: `20.0`)
- `battery_drainage_per_poi` (default: `5.0`)
- `battery_safety_margin` (default: `10.0`)
- `low_battery_threshold` (default: `35.0`)
- `critical_battery_threshold` (default: `20.0`)
- `moderate_time_pressure_ratio` (default: `1.1`)
- `high_time_pressure_ratio` (default: `1.3`)

## Run with custom parameters

```bash
ros2 run simulated_plan_component simulated_plan_component --ros-args \
  -p battery_drainage_per_poi:=6.0 \
  -p battery_safety_margin:=12.0 \
  -p low_battery_threshold:=40.0 \
  -p high_time_pressure_ratio:=1.25
```

## Launch file

```bash
ros2 launch simulated_plan_component simulated_plan_component.launch.py
```

Override launch arguments:

```bash
ros2 launch simulated_plan_component simulated_plan_component.launch.py \
  battery_drainage_per_poi:=6.0 low_battery_threshold:=40.0 high_time_pressure_ratio:=1.25
```
