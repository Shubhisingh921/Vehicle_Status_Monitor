from setuptools import setup

package_name = 'mqtt_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'paho-mqtt>=1.5.0',
        'rclpy',
    ],
    zip_safe=True,
    author='Vijay',
    author_email='saurabh@futuristicbots.com',
    description='A ROS 2 package bridging ROS topics and MQTT.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mqtt_bridge_node = mqtt_bridge.mqtt_bridge_node:main',
        ],
    },
)

