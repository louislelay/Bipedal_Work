# Bipedal Wheeled Robot

## Hardware

Modelized on Solidworks, every parts are to be 3d printed. And this is the list of the material : 

 - 4 Servo Motors : Miuzei MG996r
 - 2 DC Motor Arduino with encoder
 - 2 Wheels
 - 1 Raspberry Pi OR Jetson Nano
 - 1 IMU
 - 1 L298N driver
 - 4 bearrings
 - M3, M4 and M5 bolts and nuts

![CAD model](medias/bipedal_robot_cad.png)

![Real Robot](medias/bipedal_robot_cad.png)

## Software

Programmed using ROS2 and python. Created 4 packages.

### Package servo_controller

Subscribe from a topic name 'servo_command' and get an information under the format : "angle_hl:angle_hr:angle_kl:angle_kr"

To test it, open a first terminal and type :

```
cd GO/TO/PATH/Bipedal_Work/ros2_bipedal_ws/
source /opt/ros2/humble/setup.bash
source install/setup.bash
run servo_controller servo_controller_node
```

And in a second one :
```
cd GO/TO/PATH/Bipedal_Work/ros2_bipedal_ws/
source /opt/ros2/humble/setup.bash
source install/setup.bash
ros2 topic pub /servo_command std_msgs/msg/String "{data: "90:90:90:90"}"
```

