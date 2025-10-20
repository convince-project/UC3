# Cartesian Pointing Component — README

A compact guide to build, run, and understand the geometry behind the pointing behavior.

---

## Build & Run

```bash
colcon build --base-paths src/* --packages-up-to cartesian_pointing_component
source install/setup.bash
ros2 run cartesian_pointing_component cartesian_pointing_component
```

Depends on:

* ROS 2 (rclcpp, tf2, tf2_ros)
* YARP (ports + Map2D_nwc_yarp)
* Eigen, nlohmann/json

Make sure the Cartesian controllers are running and their RPC ports exist:

* `/cartesian-control/left_arm/rpc:i`
* `/cartesian-control/right_arm/rpc:i`

---

## ROS 2 API

**Service**: `/CartesianPointingComponent/PointAt`

```text
Request : string target_name   # name of the object in Map2D
Response: bool   is_ok
          string error_msg
```

The callback is **non‑blocking**: the service returns immediately and the task runs in a detached thread.

---

## YARP RPC expected (controller side)

The component sends these commands to the controller RPC port:

* `stop`
* `go_to_pose x y z qx qy qz qw duration`
* `is_motion_done`
* `get_pose`  → used only for diagnostics (reading the final EE rotation matrix)

---

## Pointing Strategy (high‑level)

1. **Lookup target** from Map2D and transform from `map` to `mobile_base_body_link` using TF. If TF is missing, the map coordinates are used as base coordinates.

2. **Choose the arm** (LEFT/RIGHT) by combining torso‑relative Y position (prefer the arm on the same side) and current hand distance to the target. Only the best arm is used; there is **no arm switching** mid‑task.

3. **Reach point**: move the hand along the line from shoulder to target, but clamp to a sphere with radius `reachRadius` and a minimum distance `minDist`.

4. **Orientation**: aim the **hand Z axis toward the target** (remember: in the robot hand frame, +Z points *back* toward the robot, so we use `-dir`). Keep a natural gesture by making the **palm (Y axis) face down**. Optionally apply a fixed **roll compensation** per arm to account for different palm frames (e.g., LEFT needs a π roll).

5. Send a **single** `go_to_pose` with `(position, quaternion, duration)` and poll `is_motion_done` until completion.

---

## Math (compact, copy‑friendly)

### Reach point on the shoulder→target line (clamped)

```
# vectors in base frame
u_hat = normalize( p_target - p_shoulder )
R     = min( reachRadius, max( |p_target - p_shoulder|, minDist ) )
L     = min( |p_target - p_shoulder|, R )
p_goal = p_shoulder + L * u_hat
```

### Orientation (palm down, Z toward target)

```
# +Z of the hand frame points back toward the robot, so we aim -dir to look outward
z_ee = -u_hat

# world/base "down" direction
down = (0, 0, -1)

# project 'down' on the plane orthogonal to z_ee to get palm Y (palm facing down)
y_ee = normalize( down - (down·z_ee)*z_ee )

# right‑hand basis
x_ee = normalize( y_ee × z_ee )

# rotation matrix and quaternion
R_ee = [ x_ee | y_ee | z_ee ]   # columns
q_nat = quat( R_ee )

# optional per‑arm roll (to align the palm frame), applied about hand Z
q_cmd = q_nat ⊗ quat(axis=Z_ee, angle=rollBias)
```

### One-shot trajectory

```
go_to_pose( p_goal, q_cmd, duration )
# then poll is_motion_done every kPollMs until true or timeout
```

### Final diagnostic (optional)

```
# angle between commanded q_cmd and measured q_ee (from get_pose)
θ = 2*acos( | <q_cmd, q_ee> | )   # radians
```

---

## Tunables parameters

* `reachRadius`  (default ~0.60 m): spherical reach bound from the shoulder.
* `minDist`      (default ~0.10 m): avoid singular/too‑close configurations.
* `left_roll_bias_rad`  (default π): extra roll for LEFT palm frame.
* `right_roll_bias_rad` (default 0): extra roll for RIGHT palm frame.
* Trajectory: duration and polling/timeout are constants in the source (see `kTrajDurationSec`, `kPollMs`, `kTimeoutMs`).

---

## Degenerate cases (handled)

* **Target at shoulder**: use identity quaternion and shoulder position to avoid NaNs.
* **Down ∥ z_ee**: if the projection length is tiny, pick any axis not collinear with `z_ee` (e.g., X or Y), then re‑orthogonalize.
* **Numerical tiny cross**: if `x_ee` is almost zero after the cross, use `z_ee.unitOrthogonal()` and recompute `y_ee` as `z_ee × x_ee`.

---

## Troubleshooting

* **No reply to service**: check the node is running and Map2D has the object.
* **TF warnings (map→base)**: start the TF broadcasters. Without TF the node will still run using map coords as base.
* **`go_to_pose` rejected**: controller not in `Stop` or pose invalid. The node sends `stop` before each goal; verify controller state and limits.
* **Timeout waiting motion**: tune `kTimeoutMs`, inspect joint limits/obstacles, or reduce target distance.
* **Left palm looks flipped**: increase/decrease `left_roll_bias_rad` (typical value π).

---

## License

BSD‑3‑Clause. Copyright © 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia.
