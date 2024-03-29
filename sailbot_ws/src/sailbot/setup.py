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
        ('lib/'+package_name+'/telemetry_messages/python/', glob('sailbot/telemetry_messages/python/*')),
        ('lib/'+package_name+'/trim_tab_messages/python/', glob('sailbot/trim_tab_messages/python/*')),
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
            'airmar_reader_non_lifecycle = sailbot.airmar_reader_non_lifecycle:main',
            'pwm_controller = sailbot.pwm_controller:main',
            'control_system = sailbot.control_system:main',
            'trim_tab_comms = sailbot.trim_tab_comms:main',
            'battery_monitor = sailbot.batteryMonitor:main',
            'ballast_control = sailbot.ballast_control:main',
            'computer_vision = sailbot.computer_vision:main',
            'network_comms = sailbot.network_comms:main',
            'state_manager = sailbot.state_manager:main',
            'heading_manager = sailbot.heading_controller:main'
        ],
    },
)