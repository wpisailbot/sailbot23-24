from setuptools import find_packages, setup
import os
from glob import glob

def glob_exclude(directory, exclude_patterns=[]):
    """ Glob files in directory, excluding those matching any pattern in exclude_patterns """
    files = glob(os.path.join(directory, '*'))
    for pattern in exclude_patterns:
        excluded_files = glob(os.path.join(directory, pattern))
        files = [f for f in files if f not in excluded_files]
    return files

package_name = 'sailbot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*.py')),
        ('share/' + package_name + '/config/', glob('config/*.yaml')),
        ('share/' + package_name +'/maps/', glob('maps/*.png')),
        ('lib/'+package_name+'/telemetry_messages/python/', glob_exclude('sailbot/telemetry_messages/python/*',  ['__pycache__'])),
        ('lib/'+package_name+'/trim_tab_messages/python/', glob_exclude('sailbot/trim_tab_messages/python/*',  ['__pycache__'])),
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
            'esp32_comms = sailbot.esp32_comms:main',
            'battery_monitor = sailbot.batteryMonitor:main',
            'ballast_control = sailbot.ballast_control:main',
            'computer_vision = sailbot.computer_vision:main',
            'network_comms = sailbot.network_comms:main',
            'state_manager = sailbot.state_manager:main',
            'heading_controller = sailbot.heading_controller:main',
            'heading_controller_vf = sailbot.heading_controller_vf:main',
            'path_follower = sailbot.path_follower:main',
            'path_follower_vf = sailbot.path_follower_vf:main', 
            'main_behavior = sailbot.behaviors.sailbot_behavior:main',
            'wind_smoother = sailbot.wind_smoother:main',
            'buoy_detection = sailbot.buoy_detection:main',
            'fake_movement = sailbot.fake_movement:main',
            'heading_select = sailbot.heading_select:main',
            'search_rescue_vf = sailbot.search_rescue_vf:main',
        ],
    },
)