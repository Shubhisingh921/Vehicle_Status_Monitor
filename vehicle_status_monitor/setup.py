from setuptools import find_packages, setup

package_name = 'vehicle_status_monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name], exclude=['test']),  # Include only the package directory
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),  # Ensure the resource folder exists
        ('share/' + package_name, ['package.xml']),  # Ensure package.xml exists
    ],
    install_requires=[
        'setuptools',  # Ensure this is listed
        'rclpy',       # Add rclpy as a dependency for ROS 2 nodes
        'std_msgs'     # Include any other required ROS message packages
    ],
    zip_safe=True,
    maintainer='vijay',
    maintainer_email='saurabh@futuristicbots.com',
    description='A ROS 2 package for monitoring vehicle status.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'monitor = vehicle_status_monitor.monitor:main',  # Entry point for the monitor node
        ],
    },
)

