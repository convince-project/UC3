# How to launch MADAMA tour in SIM

## Manually

### 1. Required Images

To run the MADAMA tour in simulation, you need the following Docker images:

1. **Navigation pipeline (new gazebo and navigation stack):**
    
    *elandini84/r1images:tourSim2_ubuntu24.04_jazzy_devel*
    
2. **Speech pipeline:**
    
    *elandini84/r1_talk:ub24.04_vcpkg_gccpp_v2.33*
    
3. **Components, skills, and behavior tree:**
    
    *ste93/convince:ubuntu_22.04_qt_6.8.3_sim_stable_new_robot_yarp_3.12.1*

---

### 2. Launch Procedure

### Step 1: Navigation Pipeline

> 💡 Before launching the Navigation be sure that all the following repositories are up-to-date: **tour-guide-robot**, **gz-sim-yarp-plugin**, **navigation**, **r1-models**.
> 

In the first image, start the **yarpserver** and the **yarpmanager**.

Then, launch a **yarprun** named **/console**.

Within the yarpmanager, you should find the application called **Navigation_ROS2_R1Mk3_SIM**.

Launch all the modules in this application **following the order specified by their IDs**.

> 💡 After completing this step, you can verify that everything is running correctly by visualizing the system in RViz.
> 

---

### Step 2: Speech Pipeline

In the second image, launch a **yarprun** named **/console-llm**.

Make sure that the *tour-guide-robot* repository is **up to date** and **properly built** before proceeding.

---

### Step 3: CONVINCE Pipeline

In the third image, start a **yarprun** named **/bt** and then launch the **yarpmanager**.

Here, you should find an application called **convince_bt_sim**.

Launch all the modules **in the order specified by the application**, and then connect all the components.

> 💡 Ensure that all dialog-related connections are correctly established.
> 

> ❗ If you want to interact with the robot using the keyboard, **uncomment** lines 156 and 160 in the file DialogSkill.cpp.
> 

---

### Step 4: Start the Tour

Once all pipelines are running, you can start the tour by executing the following commands:

```bash
ros2 service call /DialogComponent/SetLanguage dialog_interfaces/srv/SetLanguage "{language: \"it-IT\"}"

ros2 service call /AllowedToMoveComponent/SetAllowedToMove allowed_to_move_interfaces/srv/SetAllowedToMove is_allowed_to_move:\ true

```

## Using docker compose

From this directory run `docker compose -f docker-compose.yml up'.

Launch all the modules in the application **Navigation_ROS2_R1Mk3_SIM** **following the order specified by their IDs**.

Launch all the modules in the application **convince_bt_sim** **in the order specified by the application**, and then connect all the components.

Then [Start the Tour](#step-4-start-the-tour).

> 💡 It is still recommended to read through the manual steps to read other tips.
> 
