# Task 3: ROS 2 & Gazebo Simulation Guide

This document explains the architecture of the Task 3 simulation, detailing the purpose of each file, how ROS 2 communicates with Gazebo Harmonic, and the exact commands needed to start the simulation from scratch.

---

## 1. Directory Structure and File Roles

The Task 3 implementation is built as a ROS 2 workspace (`task3_ros2_ws`) containing a single package named `asmc_ftc_octorotor`. 

Here is what each critical file does:

### The Python Nodes (`src/asmc_ftc_octorotor/asmc_ftc_octorotor/`)
* **`asmc_node.py` (The Brain):** Contains the Adaptive Sliding Mode Controller. It reads the drone's current 3D position (Odometry) and calculates the total Thrust and Torques required to hover at 5 meters.
* **`ca_node.py` (The Distributor):** Contains the Control Allocation logic. It takes the total Thrust and Torques from the ASMC node and uses a mathematical matrix (Pseudo-Inverse) to calculate exactly how fast each of the 8 individual motors needs to spin to achieve that movement.
* **`fault_injector_node.py` (The Saboteur):** A timer-based node that mimics hardware failure. Exactly 20 seconds into the flight, it commands Motor 1 to fail completely (0%) and Motor 5 to lose power (60%).

### The Gazebo Models (`src/asmc_ftc_octorotor/models/octorotor/`)
* **`model.sdf`:** The Gazebo physical definition of the drone. It defines the weight (2.0 kg), the moment of inertia, the placement of the 8 motors, the direction they spin (CW vs CCW), and loads the 3D meshes so you can see the drone on the screen.
* **`octorotor.world`:** The 3D environment file. It defines the gravity (-9.81), the sun/lighting, and the ground plane the drone takes off from.

### The Launch File (`src/asmc_ftc_octorotor/launch/`)
* **`sim.launch.py`:** The master script. When executed, it automatically opens the Gazebo 3D window, loads the drone, starts the bridge, and boots up all three Python nodes simultaneously.

---

## 2. How ROS 2 and Gazebo Interact

ROS 2 and Gazebo are completely separate software programs. They communicate with each other using a translator called the **`ros_gz_bridge`**.

1. **Sensing (Gazebo $\rightarrow$ ROS 2):** Gazebo calculates the physics of the drone falling or flying. It outputs this data to the bridge, which translates it into a ROS 2 `Odometry` message. The `asmc_node.py` reads this to know where the drone is.
2. **Thinking (ROS 2 Internal):** The `asmc_node.py` passes "Virtual Controls" to `ca_node.py`, which then calculates the 8 motor speeds.
3. **Acting (ROS 2 $\rightarrow$ Gazebo):** The `ca_node.py` sends an `Actuators` message to the bridge. The bridge translates this into Gazebo's format and feeds it into the `model.sdf` motor plugins, which physically spin the 3D propellers, causing lift.

---

## 3. How to Launch the Simulation (From Scratch)

If you have closed all terminals and need to run the drone simulation again, follow these exact steps.

**Step 1: Open a new WSL (Ubuntu) terminal.**

**Step 2: Clean up old ghost processes.** 
If Gazebo crashed previously, invisible background processes might ruin your next flight. Run this command to kill them:
```bash
killall -9 asmc_node ca_node fault_injector_node gz ruby parameter_bridge python3 || true
```

**Step 3: Navigate to the ROS 2 Workspace.**
```bash
cd ~/OneDrive/ftc_logs/IISC_INTERNSHIP/task3_ros2_ws
```

**Step 4: Source the ROS 2 Jazzy environment.**
```bash
source /opt/ros/jazzy/setup.bash
```

**Step 5: Source the local workspace.**
(If you have made edits to the python files, run `colcon build` first, otherwise just source).
```bash
source install/setup.bash
```

**Step 6: Launch the Simulation!**
```bash
ros2 launch asmc_ftc_octorotor sim.launch.py
```
*Wait 5 to 10 seconds for the Gazebo UI window to appear. The drone will immediately lift off the ground, stabilize at 5 meters, and encounter motor failures at the 20-second mark.*
