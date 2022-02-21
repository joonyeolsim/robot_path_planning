from setuptools import setup
from glob import glob
import os

package_name = 'robot_path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.model')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/aws_warehouse/*.pgm')),
        (os.path.join('share', package_name, 'maps'), glob('maps/aws_warehouse/*.png')),
        (os.path.join('share', package_name, 'maps'), glob('maps/aws_warehouse/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='delicate',
    maintainer_email='1219530@inha.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_path_planning = robot_path_planning.robot_path_planning:main'
        ],
    },
)
