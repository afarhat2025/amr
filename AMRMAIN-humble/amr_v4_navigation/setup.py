from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_v4_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),

        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amr_1',
    maintainer_email='amr_1@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_nav = amr_v4_navigation.nav_rob:main",
            "line_follower = amr_v4_navigation.linefollower:main",
            "charging = amr_v4_navigation.chargingstrategy:main",
        ],
    },
)
