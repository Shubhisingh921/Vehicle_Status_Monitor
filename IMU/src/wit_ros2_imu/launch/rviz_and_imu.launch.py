from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    imu_node = Node(
        package='wit_ros2_imu',           #
        executable='imu_node',            
        name='rviz_and_imu',              
        output='screen',                  
        parameters=[                      
            {'port': '/dev/ttyUSB0'},
            {'baud': 9600}
        ],
        remappings=[('/wit/imu', '/imu/data')]  
    )

    return LaunchDescription([imu_node])
