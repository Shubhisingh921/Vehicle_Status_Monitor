from setuptools import find_packages, setup

package_name = 'pyside2_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    install_requires=[
        'setuptools',
        'paho-mqtt',
        'rclpy',
        'PySide2'
    ],
    zip_safe=True,
    maintainer='vijay',
    maintainer_email='saurabh@futuristicbots.com',
    description='A GUI for vehicle status monitoring',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pyside2_gui = pyside2_gui.gui_node:main',
        ],
    },
)
