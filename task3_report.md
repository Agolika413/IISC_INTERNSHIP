# Task 3 Implementation & Physics Failure Report

This document details the implementation of Task 3 (ROS2 + Gazebo Harmonic integration) and provides a deep-dive analysis into the physics-based "Free-Fall" crashing issue we encountered, contrasting it with the simplified Python simulation from Task 1.

## 1. Task 3 Implementation Overview

The goal of Task 3 was to port the **Adaptive Sliding Mode Control (ASMC)** and **Control Allocation** algorithms from a pure mathematical Python script into a highly realistic, physics-driven 3D simulation environment using **ROS 2 Jazzy** and **Gazebo Harmonic**.

### Architecture
We developed a modular ROS2 workspace (`task3_ros2_ws`) consisting of three primary nodes that communicate over ROS topics:

1. **`asmc_node.py`**: The "Brain". It reads the drone's current Odometry (position and velocity) from Gazebo, compares it against the desired trajectory (5 meters altitude), and calculates the required total Thrust (Z-force) and Torques (Roll, Pitch, Yaw) needed to correct any errors.
2. **`ca_node.py`**: The "Distributor". It takes the virtual controls (Thrust + Torques) and uses a **Pseudo-Inverse Mathematical Matrix** to distribute those forces evenly across the 8 physical motors of the coaxial octorotor. 
3. **`fault_injector_node.py`**: The "Saboteur". It monitors the simulation clock. For the first 20 seconds, it reports all motors as 100% healthy. Exactly at $t = 20.0s$, it kills Motor 1 (0% health) and damages Motor 5 (60% health), forcing the ASMC to dynamically adapt in real-time.

---

## 2. The "Free-Fall" Failure Analysis

During early testing, we observed a catastrophic failure pattern:
> **The drone would rocket up from the ground, reach 5 meters, immediately turn off its motors, tumble out of the sky, and crash violently into the ground.**

### Why did it work in Task 1 (Python) but fail in Task 3 (Gazebo)?

**The Python Simulation Illusion**
In Task 1, the simulation was purely mathematical. When the controller was given a "Step Input" to jump instantly from 0 to 5 meters, it commanded massive thrust. When it hit 5 meters, it was moving upwards very fast, so the math commanded **0 Newtons (0N)** of thrust to brake. In Python, this simply resulted in the altitude value decreasing gracefully. Python did not simulate 3D rigid-body dynamics or gravity-induced tumbling.

**The Cold Reality of Gazebo Physics**
Gazebo runs a rigid-body physics engine. A multirotor drone steers and balances itself *exclusively* by varying the RPM of its spinning blades. 
When the ASMC controller realized it was overshooting 5 meters, it panicked and commanded `Thrust = 0`. 
1. **Total thrust dropped to 0N**, meaning all 8 motors completely stopped spinning.
2. Because the motors were off, the drone had **Zero Attitude Authority**. It became a dead weight in free-fall.
3. As it fell, aerodynamic drag and microscopic imbalances caused it to tip over.
4. Once it fell below 5 meters, the ASMC violently turned the motors back on to maximum power. 
5. However, because the drone had tipped over in free-fall, the "upward" thrust was now pointing sideways. The drone essentially turned into a horizontal missile and obliterated itself against the ground plane.

---

## 3. The Resolution

To solve this physics reality clash, three major fixes were implemented:

### A. The "Idle Thrust" Safety Net
We modified `asmc_node.py` to enforce a hard lower bound on thrust.
```python
# Prevent thrust from dropping to 0 (which causes total loss of attitude control)
msg.force.z = float(np.clip(nu[0] * self.m, 5.0, 35.0))
```
Instead of shutting down completely, the drone is now forced to maintain a minimum of **5.0 Newtons** of thrust. Because 5N is far lower than the ~19.6N required to hover, the drone is still able to "fall" and brake its ascent. However, keeping the motors spinning at 5N acts as an "idle speed," allowing the drone to actively balance its Pitch and Roll *while* it falls, perfectly preventing the tumble.

### B. Balanced Control Allocation (Pseudo-Inverse)
We replaced the aggressive `lsq_linear` constraint solver with a direct `np.linalg.pinv` (Moore-Penrose Pseudo-Inverse) calculation in `ca_node.py`. The previous linear solver was attempting to aggressively zero out certain motors to meet constraints, which exacerbated the flipping. The pseudo-inverse guarantees the "minimum norm" solution, meaning it distributes the required effort as evenly as mathematically possible across all 8 healthy motors.

### C. Eradicating Ghost Nodes
We discovered that rapid crashing was also being caused by "Ghost Nodes". Previous failed executions of the simulation had left invisible Python processes running in the background. An old `fault_injector` (which had already passed its 20-second timer) was fighting with the new `fault_injector`. 
* Node A was commanding: "Motors Healthy!"
* Node B was commanding: "Motors Dead!"
This resulted in the motors violently oscillating between 0% and 100% health hundreds of times per second. Implementing a strict `pkill -9` cleanup command prior to launching Gazebo permanently resolved this instability.
