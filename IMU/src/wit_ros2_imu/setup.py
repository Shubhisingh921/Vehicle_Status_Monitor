from setuptools import setup

package_name = 'wit_ros2_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' +package_name, ['launch/rviz_and_imu.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vijay',
    maintainer_email='saurabh@futuristicbots.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
    'console_scripts': [
        'imu_node = wit_ros2_imu.imu_node:main',
        ],
    },
)
