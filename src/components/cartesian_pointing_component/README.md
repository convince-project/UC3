# Cartesian Pointing Component — README

A compact guide to build, run, and understand the geometry behind the pointing behavior.

---

## Build & Run

```bash
colcon build --base-paths src/* --packages-up-to cartesian_pointing_component
source install/setup.bash

# Option A: use the UC3 repository file (inside this repo)
ros2 run cartesian_pointing_component cartesian_pointing_component \
    --ros-args -p map2d_locations_file:=$PWD/config/locations.ini

# Or pass your own absolute path to a file with an Objects: section
# ros2 run cartesian_pointing_component cartesian_pointing_component \
#   --ros-args -p map2d_locations_file:=/absolute/path/to/locations.ini
```

Depends on:

* ROS 2 (rclcpp, tf2, tf2_ros)
* YARP (ports + Map2D_nwc_yarp)
* Eigen

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

## Motion status API

- Service: `/CartesianPointingComponent/IsMotionDone` (cartesian_pointing_interfaces/IsMotionDone)
    - Risposta:
        - `is_done=false` finché un movimento è in corso
        - `is_done=true` quando il movimento è finito (o non ce n’è nessuno attivo)
        - `is_ok` indica l’esito della query (true se la lettura è andata a buon fine)

---

## Map2D input file (Objects only)

At startup, the component imports the Objects section from the file passed via the ROS parameter `map2d_locations_file`.

- Supported lines in the `Objects:` section:
    - `name ( map x y z [roll pitch yaw "desc"] )`
    - `name map x y z [anything...]`
- Only `map` and `x y z` are used. Orientation (roll/pitch/yaw) and description are ignored.
- Empty lines and lines starting with `#` are ignored.

Example:

```
Objects:
inizio            map 3.0   -0.3  1.5  "punto di inizio tour"
quadro_principale map 1.058 -3.0  1.505 "opera principale"
scultura_est      map 4.0    5.0  8.0
dipinto_ovest     map 7.0    7.0  4.0
```

At startup you should see a summary like:

```
[INFO] [...] Available objects (4)
[INFO] [...]  - inizio: x=3.000 y=-0.300 z=1.500
[INFO] [...]  - quadro_principale: x=1.058 y=-3.000 z=1.505
[INFO] [...]  - scultura_est: x=4.000 y=5.000 z=8.000
[INFO] [...]  - dipinto_ovest: x=7.000 y=7.000 z=4.000
```

Notes:
- Only Objects are imported by this component. Targets must exist in the `Objects` section; there is no fallback to `Locations`.
    At the moment no default locations.ini is installed with the package; pass an explicit file path (e.g., the repo-level `config/locations.ini`).

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
* `map2d_locations_file` (required): absolute path to a file containing an `Objects:` section.
* Trajectory: duration and polling/timeout are constants in the source (see `kTrajDurationSec`, `kPollMs`, `kTimeoutMs`).

---

## Degenerate cases (handled)

* **Target at shoulder**: use identity quaternion and shoulder position to avoid NaNs.
* **Down ∥ z_ee**: if the projection length is tiny, pick any axis not collinear with `z_ee` (e.g., X or Y), then re‑orthogonalize.
* **Numerical tiny cross**: if `x_ee` is almost zero after the cross, use `z_ee.unitOrthogonal()` and recompute `y_ee` as `z_ee × x_ee`.

---

## Troubleshooting

* **No reply to service**: check the node is running and Map2D has the object.
* **No objects listed at startup**: verify `map2d_locations_file` points to a readable file and that it contains a valid `Objects:` section.
* **TF warnings (map→base)**: start the TF broadcasters. Without TF the node will still run using map coords as base.
* **Timeout waiting motion**: tune `kTimeoutMs`, inspect joint limits/obstacles, or reduce target distance.
* **Left palm looks flipped**: increase/decrease `left_roll_bias_rad` (typical value π).

---

## License

BSD‑3‑Clause. Copyright © 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia.
