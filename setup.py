from setuptools import setup

package_name = 'pose_pubsub'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@email.com',
    description='Publish PoseStamped from CSV and listen to it',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pose_publisher = pose_pubsub.pose_publisher:main',
            'pose_subscriber = pose_pubsub.pose_subscriber:main',
            'pose_to_path = pose_pubsub.pose_to_path:main',
        ],
    },
)