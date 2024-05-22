To test in rviz

# Terminal 1
. ~/ros2_ws/install/setup.bash # setup.zsh if you use zsh instead of bash
ros2 launch bipedal_bringup bringup.launch.py joint_hardware_connected:=false rviz:=true robot_name:=bipedal

# Terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Then control robot dog with the keyboard


sudo apt install pigpio python3-pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
