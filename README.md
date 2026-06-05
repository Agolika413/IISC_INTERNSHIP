# ASMC-FTC Octorotor Project

This repository contains the reproduction of the paper *"An Adaptive Fault-Tolerant Sliding Mode Control Allocation Scheme for Multirotor Helicopter Subject to Simultaneous Actuator Faults"*.

## Project Structure

1. **`task1_python_simulation/`**: Pure Python implementation of the Octorotor dynamics, ASMC, NSMC, LQR controllers, and the Control Allocation module with realistic motor saturation. Generates plots matching Figs. 5-11 from the paper.
2. **`task2_nmpc_simulation/`**: Comparison of the high-level ASMC against a Nonlinear Model Predictive Controller (NMPC) using CasADi.
3. **`task3_ros2_ws/`**: ROS2 + Gazebo Sim implementation of the ASMC-FTC framework.

## Task 1 & 2 Setup (Python)
Dependencies: `numpy`, `matplotlib`, `scipy`, `casadi`.
To run the python simulations:
```bash
cd task1_python_simulation
python3 run_task1.py
```
```bash
cd task2_nmpc_simulation
python3 run_task2.py
```

## Task 3 Setup (ROS2 Jazzy + Gazebo Sim)
This package is configured for ROS2 Jazzy and Gazebo Sim. It uses a Python-based implementation for the `asmc_node`, `ca_node` (Control Allocation), and `fault_injector` to match the Python physics from Tasks 1 and 2 seamlessly.

**Building:**
```bash
cd task3_ros2_ws
colcon build
source install/setup.bash
```

**Running the Simulation:**
```bash
ros2 launch asmc_ftc_octorotor sim.launch.py
```

**Architecture:**
- **`asmc_node`**: Subscribes to `/odometry` from Gazebo, computes virtual control inputs using Adaptive Sliding Mode Control, and publishes to `/virtual_controls`.
- **`ca_node`**: Subscribes to `/virtual_controls` and `/fault_injection`. Redistributes thrust across the 8 motors using pseudo-inverse control allocation.
- **`fault_injector`**: Timer-based node that publishes simulated loss-of-effectiveness values $L_i$ to `/fault_injection` at $t=20s$ to replicate Scenario 2.
- **Gazebo Sim**: Simulates the nonlinear physics and multicopter aerodynamics.
