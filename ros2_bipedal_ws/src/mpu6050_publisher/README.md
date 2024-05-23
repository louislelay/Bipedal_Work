1. install the python-smbus package :

sudo apt install python3-smbus

2a. Install this package from PyPi repository :

pip install mpu6050-raspberrypi

Or:

2b. Clone the repository and run setup.py :

git clone https://github.com/m-rtijn/mpu6050.git
python setup.py install



ros2 run mpu6050_publisher mpu6050_node
