setup:
```bash
docker start ros2_jetson
docker exec -it ros2_jetson bash
source /opt/ros/galactic/setup.bash
cd ~/ros2_ws
source install/setup.bash

ros2 run jetson_motor_controller motor_controller_node

```

build:
```bash
colcon build --packages-select jetson_motor_controller
source install/setup.bash
```

testing:
```bash
docker exec -it ros2_jetson bash
source /opt/ros/galactic/setup.bash
cd ~/ros2_ws
source install/setup.bash

ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```
