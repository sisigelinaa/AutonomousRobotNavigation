from setuptools import find_packages, setup
import os        
from glob import glob  

package_name = 'my_bot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'models', 'turtlebot3_radar'), glob(os.path.join('models', 'turtlebot3_radar', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytesz',
        ],
    },
    entry_points={
        'console_scripts': [
        'planner_controller = my_bot_controller.planner_controller:main',
        'visualiser = my_bot_controller.visualiser:main',
        'scenario_runner = my_bot_controller.scenario_runner:main',
        'metrics_logger = my_bot_controller.metrics_logger:main',
        'breathing_detector = my_bot_controller.breathing_detector:main',
        'breathing_detector_realdata = my_bot_controller.breathing_detector_realdata:main',
        ],
    },
)