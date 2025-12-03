# ...existing code...
from setuptools import setup
import os
from glob import glob

package_name = 'pose_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Install RViz configs
        ('share/' + package_name + '/config/rviz', glob('config/rviz/*.rviz')),
        # Optional: data files
        ('share/' + package_name + '/data', glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='helen',
    maintainer_email='helen@example.com',
    description='Publish CSV poses and visualize as paths',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_to_path = pose_pubsub.pose_to_path:main',
            'csv_pose_player = pose_pubsub.csv_pose_player:main',
            'pose_publisher = pose_pubsub.pose_publisher:main',
            'pose_subscriber = pose_pubsub.pose_subscriber:main',
        ],
    },
)