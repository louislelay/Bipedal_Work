1. install the python-smbus package :

sudo apt install python3-smbus

2a. Install this package from PyPi repository :

pip install mpu6050-raspberrypi

Or:

2b. Clone the repository and run setup.py :

git clone https://github.com/m-rtijn/mpu6050.git
python setup.py install



ros2 run mpu6050_publisher mpu6050_node


    VCC (Power) to 3.3V or 5V Power Pin
    GND (Ground) to Ground Pin
    SCL (Clock) to GPIO Pin 3 (I2C1 SCL)
    SDA (Data) to GPIO Pin 2 (I2C1 SDA)