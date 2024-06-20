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

After installing ROS2. Source ROS2 and then source the workspace.

### Package servo_controller

Subscribe from a topic name 'servo_command' and get an information under the format : "angle_hl:angle_hr:angle_kl:angle_kr"

To test it, open a first terminal and type :

'''ros2 run servo_controller servo_controller_node'''
