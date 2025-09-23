# MoveIt Stacker Insertion Launch Files

This directory contains launch files for running the MoveIt stacker insertion demo with the Franka FR3 robot.

## Launch Files

### 1. `moveit_stacker_insertion.launch.py`
**For real hardware operation**

This launch file starts MoveIt with all necessary components and runs the stacker insertion script on real hardware.

**Usage:**
```bash
ros2 launch franka_fr3_moveit_config moveit_stacker_insertion.launch.py robot_ip:=<YOUR_ROBOT_IP>
```

**Parameters:**
- `robot_ip`: IP address of the Franka robot (required)
- `use_fake_hardware`: Set to `false` for real hardware (default: `false`)
- `namespace`: Robot namespace (default: empty)
- `start_stacker_script`: Whether to start the stacker insertion script (default: `true`)

### 2. `moveit_stacker_insertion_sim.launch.py`
**For simulation/testing**

This launch file starts MoveIt with fake hardware for simulation and testing purposes.

**Usage:**
```bash
ros2 launch franka_fr3_moveit_config moveit_stacker_insertion_sim.launch.py
```

**Parameters:**
- `robot_ip`: IP address (default: `172.16.0.2`, not used in simulation)
- `use_fake_hardware`: Set to `true` for simulation (default: `true`)
- `namespace`: Robot namespace (default: empty)
- `start_stacker_script`: Whether to start the stacker insertion script (default: `true`)

## What the Launch Files Include

Both launch files start the following components:

1. **MoveIt Move Group** - Core planning and execution
2. **RViz2** - Visualization with MoveIt configuration and RvizVisualToolsGui
3. **Robot State Publisher** - Publishes robot transforms
4. **ROS2 Control Node** - Hardware interface
5. **Joint State Publisher** - Publishes joint states
6. **Controller Manager** - Manages robot controllers
7. **Franka Gripper** - Gripper control
8. **MoveIt Stacker Insertion Script** - Your custom demo script

## RvizVisualToolsGui

The RViz configuration now includes the **RvizVisualToolsGui** panel, which provides interactive buttons for the MoveIt visual tools prompts. This allows you to:

- Click "Next" to continue with planning and execution steps
- Interact with the MoveIt visual tools prompts from your script
- Control the flow of the stacker insertion demo through the GUI

The GUI will appear as a panel in RViz when you launch the demo, and you can use it to step through the planning and execution phases of your script.

## Prerequisites

1. **Build the packages:**
   ```bash
   cd /home/david/franka_ros2_ws
   colcon build --packages-select franka_example_controllers franka_fr3_moveit_config
   source install/setup.bash
   ```

2. **For real hardware:**
   - Ensure the robot is connected and accessible via network
   - Have the correct robot IP address
   - Ensure proper safety setup and emergency stops

3. **For simulation:**
   - No additional setup required
   - The launch file will use fake hardware automatically

## Troubleshooting

1. **Script doesn't start:** The stacker insertion script has a 5-second delay to allow MoveIt to fully initialize. Wait for this delay.

2. **"robot_description_semantic not found" error:** This has been fixed by passing the necessary parameters to the script in the launch file.

3. **"Unable to construct robot model" error:** This was caused by incorrect group names. The script now uses the correct Franka FR3 group names:
   - `fr3_arm` (instead of `manipulator`)
   - `fr3_hand` (instead of `gripper`)

4. **Planning fails:** Check that the robot is in a safe starting position and that the planning scene is properly loaded.

5. **Controller issues:** Ensure the robot controllers are properly loaded and the robot is in the correct mode.

6. **Mesh loading errors:** Verify that the mesh files are accessible in the `franka_description` package.

7. **Trajectory visualization not working:** The script now includes proper initialization of MoveIt visual tools and publishes the robot state to help with trajectory visualization in RViz.

8. **RvizVisualToolsGui not appearing:** Make sure the `moveit_rviz_plugin` package is installed. If the GUI panel doesn't appear, try restarting RViz or check that the topic `/rviz_visual_tools_gui` is being published.

## Customization

You can modify the launch files to:
- Change the delay before starting the stacker insertion script
- Add additional parameters
- Include other components
- Modify the RViz configuration

The stacker insertion script itself can be found in:
`franka_example_controllers/src/moveit_stacker_insertion.cpp`
