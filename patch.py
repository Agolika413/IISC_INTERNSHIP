import re
with open('task3_ros2_ws/src/asmc_ftc_octorotor/models/octorotor/model.sdf', 'r') as f:
    content = f.read()

# Fix commandSubTopic
for i in range(8):
    content = re.sub(
        r'<commandSubTopic>/actuator_commands_0</commandSubTopic>\s*<motorNumber>{}</motorNumber>'.format(i),
        '<commandSubTopic>/motor_speed/{}</commandSubTopic>\n      <motorNumber>{}</motorNumber>'.format(i, i),
        content
    )

# Fix body to be a realistic box instead of cylinder
content = re.sub(
    r'<visual name=\"base_link_visual\">\s*<geometry>\s*<cylinder>\s*<radius>0.1</radius>\s*<length>0.1</length>\s*</cylinder>\s*</geometry>\s*<material>.*?</material>\s*</visual>',
    '''<visual name=\"base_link_visual\">
        <geometry>
          <box><size>0.22 0.22 0.08</size></box>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>''',
    content,
    flags=re.DOTALL
)

# Fix arms to be sleek square tubes
for i in range(1, 5):
    content = re.sub(
        rf'<visual name=\"arm{i}_visual\">\s*<pose>(.*?)</pose>\s*<geometry><cylinder><radius>0.015</radius><length>0.23</length></cylinder></geometry>\s*<material>(.*?)</material>\s*</visual>',
        rf'<visual name=\"arm{i}_visual\">\n        <pose>\g<1></pose>\n        <geometry><box><size>0.02 0.02 0.23</size></box></geometry>\n        <material>\g<2></material>\n      </visual>',
        content
    )

# Fix rotors to use realistic meshes from Gazebo fuel
for i in range(8):
    cw = 'cw' if i in [0, 3, 4, 7] else 'ccw'
    color = '0.1 0.8 0.1 0.5' if i in [0, 2, 4, 6] else '0.8 0.1 0.1 0.5'
    
    # We replace the whole cylinder visual with the mesh
    content = re.sub(
        rf'<visual name=\"rotor_{i}_visual\"><geometry><cylinder><radius>0.1</radius><length>0.01</length></cylinder></geometry><material><ambient>{color}</ambient></material></visual>',
        rf'''<visual name="rotor_{i}_visual">
        <geometry>
          <mesh>
            <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3 UAV/4/files/meshes/propeller_{cw}.dae</uri>
            <scale>0.1 0.1 0.1</scale>
          </mesh>
        </geometry>
      </visual>''',
        content
    )

with open('task3_ros2_ws/src/asmc_ftc_octorotor/models/octorotor/model.sdf', 'w') as f:
    f.write(content)
