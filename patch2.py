import re
with open('task3_ros2_ws/src/asmc_ftc_octorotor/models/octorotor/model.sdf', 'r') as f:
    content = f.read()

for i in range(8):
    content = content.replace(f'<commandSubTopic>/motor_speed/{i}</commandSubTopic>', f'<commandSubTopic>/motor_speed_{i}</commandSubTopic>')

with open('task3_ros2_ws/src/asmc_ftc_octorotor/models/octorotor/model.sdf', 'w') as f:
    f.write(content)
