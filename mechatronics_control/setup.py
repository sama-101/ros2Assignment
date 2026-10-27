from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mechatronics_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
        os.path.join('share', package_name, 'launch'),
        glob(os.path.join('launch', '*.launch.py'))
),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mariam',
    maintainer_email='mariam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'speed_controller = mechatronics_control.speed_controller:main',
        'go_to_pose_server = mechatronics_control.go_to_pose_server:main',
        'reset_service = mechatronics_control.reset_service:main',
        'square_action_server = mechatronics_control.square_action_server:main',
    ],
},
)
