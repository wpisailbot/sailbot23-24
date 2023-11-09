from setuptools import find_packages, setup
from glob import glob

package_name = 'sailbot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.py')),
        ('lib/'+package_name+'/telemetry_messages/', glob('sailbot/telemetry_messages/python/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WPI Sailbot MQP',
    maintainer_email='gr-sailbot23-24@wpi.edu',
    description='Primary ROS package for WPI\'s autonomous sailboat MQP',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'airmar_reader = sailbot.airmar_reader:main',
            'pwm_controller = sailbot.pwm_controller:main',
            'serial_rc_receiver = sailbot.serial_rc_receiver:main',
            'control_system = sailbot.control_system:main',
            'trim_tab_comms = sailbot.trim_tab_comms:main',
            'debug_interface = sailbot.debug_interface:main',
            'battery_monitor = sailbot.batteryMonitor:main',
            'ballast_control = sailbot.ballast_control:main',
            'computer_vision = sailbot.computer_vision:main',
            'network_comms = sailbot.network_comms:main'
        ],
    },
)