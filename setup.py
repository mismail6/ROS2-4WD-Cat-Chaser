from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cat_chaser'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ADD THESE LINES:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),  # optional
        # materials
        #(os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),  # if you have world files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ismail',
    maintainer_email='ismail@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard_hold = cat_chaser.teleop_keyboard_hold:main',
            'odom_to_base = cat_chaser.odom_to_base:main',
            'twist_bridge = cat_chaser.twist_bridge:main',
            'camera_view = cat_chaser.camera_view:main',
        ],
    },
)
