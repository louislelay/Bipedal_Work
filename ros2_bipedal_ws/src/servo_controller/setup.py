from setuptools import setup

package_name = 'servo_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'servo_controller'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package to control 4 servo motors using PWM pins on a Raspberry Pi.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_controller = servo_controller:main',
        ],
    },
)
