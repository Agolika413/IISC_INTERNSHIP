import math
import os

Ld = 0.23
b_t = 2.8e-06
d_t = 7.5e-08

# We'll use the X3 UAV propeller mesh from Fuel.
# Since fuel URLs can sometimes be slow to load, we can just use the exact fuel URI.
# White color for blades, Black/Dark grey for body.

sdf = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="octorotor">
    <pose>0 0 5.0 0 0 0</pose>
    
    <link name="base_link">
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.0820</ixx><iyy>0.0845</iyy><izz>0.1377</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      
      <collision name="base_link_collision">
        <geometry><cylinder><radius>0.12</radius><length>0.08</length></cylinder></geometry>
      </collision>

      <!-- Center Body (Sleek Black Hexagon or Cylinder) -->
      <visual name="base_link_visual">
        <geometry><cylinder><radius>0.12</radius><length>0.08</length></cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse><specular>0.5 0.5 0.5 1</specular></material>
      </visual>

      <!-- Arm Front (+x) -->
      <visual name="arm_front">
        <pose>{Ld/2} 0 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.012</radius><length>{Ld}</length></cylinder></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
      <!-- Arm Right (+y) -->
      <visual name="arm_right">
        <pose>0 {Ld/2} 0 1.5708 0 0</pose>
        <geometry><cylinder><radius>0.012</radius><length>{Ld}</length></cylinder></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
      <!-- Arm Rear (-x) -->
      <visual name="arm_rear">
        <pose>-{Ld/2} 0 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.012</radius><length>{Ld}</length></cylinder></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
      <!-- Arm Left (-y) -->
      <visual name="arm_left">
        <pose>0 -{Ld/2} 0 1.5708 0 0</pose>
        <geometry><cylinder><radius>0.012</radius><length>{Ld}</length></cylinder></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
    </link>
'''

positions = [
    (Ld, 0, 0.05, 'cw', 0),
    (Ld, 0, -0.05, 'ccw', 1),
    (0, Ld, 0.05, 'ccw', 2),
    (0, Ld, -0.05, 'cw', 3),
    (-Ld, 0, 0.05, 'cw', 4),
    (-Ld, 0, -0.05, 'ccw', 5),
    (0, -Ld, 0.05, 'ccw', 6),
    (0, -Ld, -0.05, 'cw', 7)
]

for x, y, z, dir, idx in positions:
    mesh_uri = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3 UAV/4/files/meshes/propeller_cw.dae" if dir == 'cw' else "https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3 UAV/4/files/meshes/propeller_ccw.dae"
    # Note: scale 0.1 works well for X3 propellers which are natively around 1m radius. 0.1 gives 0.1m.
    sdf += f'''
    <link name="rotor_{idx}">
      <pose>{x} {y} {z} 0 0 0</pose>
      <inertial><mass>0.005</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz></inertia></inertial>
      <visual name="rotor_{idx}_visual">
        <geometry>
          <mesh>
            <uri>{mesh_uri}</uri>
            <scale>0.1 0.1 0.1</scale>
          </mesh>
        </geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>1 1 1 1</specular></material>
      </visual>
    </link>
    <joint name="rotor_{idx}_joint" type="revolute">
      <parent>base_link</parent><child>rotor_{idx}</child>
      <axis><xyz>0 0 1</xyz></axis>
    </joint>
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_{idx}_joint</jointName>
      <linkName>rotor_{idx}</linkName>
      <turningDirection>{dir}</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>2000</maxRotVelocity>
      <motorConstant>{b_t}</motorConstant>
      <momentConstant>{d_t/b_t}</momentConstant>
      <commandSubTopic>/motor_speed_{idx}</commandSubTopic>
      <motorNumber>{idx}</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    </plugin>
'''

sdf += '''
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_publish_frequency>200</odom_publish_frequency>
      <odom_topic>/model/octorotor/odometry</odom_topic>
    </plugin>
  </model>
</sdf>
'''

with open('task3_ros2_ws/src/asmc_ftc_octorotor/models/octorotor/model.sdf', 'w') as f:
    f.write(sdf)
