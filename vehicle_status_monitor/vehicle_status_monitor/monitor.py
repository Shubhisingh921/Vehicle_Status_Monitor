# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan, Imu
# from std_msgs.msg import String
# import time


# class VehicleStatusMonitor(Node):
#     def __init__(self):
#         super().__init__('vehicle_status_monitor')

#         # LaserScan Sensor Setup
#         self.laserscan_subscription = self.create_subscription(
#             LaserScan,
#             '/scan',  # Replace with your LaserScan topic
#             self.laserscan_callback,
#             10
#         )
#         self.laserscan_last_data_time = time.time()
#         self.laserscan_status = "unknown"

#         # IMU Sensor Setup
#         self.imu_subscription = self.create_subscription(
#             Imu,
#             '/imu/data_raw',  # Replace with your IMU topic
#             self.imu_callback,
#             10
#         )
#         self.imu_last_data_time = time.time()
#         self.imu_status = "unknown"

#         # Publisher to publish combined status
#         self.publisher_ = self.create_publisher(String, 'multi_sensor_status', 10)

#         # Timer to check sensor status every second
#         self.timer = self.create_timer(1.0, self.check_sensor_status)

#     def laserscan_callback(self, msg):
#         """Callback for LaserScan sensor."""
#         self.get_logger().info('LaserScan data received.')
#         self.laserscan_last_data_time = time.time()

#     def imu_callback(self, msg):
#         """Callback for IMU sensor."""
#         self.get_logger().info('IMU data received.')
#         self.imu_last_data_time = time.time()

#     def check_sensor_status(self):
#         """Check the status of both sensors."""
#         current_time = time.time()

#         # Check LaserScan Sensor
#         if current_time - self.laserscan_last_data_time > 2.0:
#             if self.laserscan_status != "malfunction":
#                 self.laserscan_status = "malfunction"
#                 self.log_and_report('LaserScan sensor: Malfunction')
#         else:
#             if self.laserscan_status != "operational":
#                 self.laserscan_status = "operational"
#                 self.log_and_report('LaserScan sensor: Operational')
        
#         # Check IMU Sensor
#         if current_time - self.imu_last_data_time > 2.0:
#             if self.imu_status != "malfunction":
#                 self.imu_status = "malfunction"
#                 self.log_and_report('IMU sensor: Malfunction')
#         else:
#             if self.imu_status != "operational":
#                 self.imu_status = "operational"
#                 self.log_and_report('IMU sensor: Operational')

#         # Publish Combined Status
#         status_message = String()
#         status_message.data = (
#             f"LaserScan: {self.laserscan_status}, IMU: {self.imu_status}"
#         )
#         self.publisher_.publish(status_message)
#         self.get_logger().info(f"Published: {status_message.data}")

#     def log_and_report(self, message):
#         """Log the message and optionally write it to a file."""
#         if "Malfunction" in message:
#             self.get_logger().warn(message)
#         else:
#             self.get_logger().info(message)

#         # Uncomment if logging to a file is necessary:
#         # with open('multi_sensor_status_report.txt', 'a') as report_file:
#         #     report_file.write(f'{time.ctime()} - {message}\n')


# def main(args=None):
#     rclpy.init(args=args)
#     monitor_node = VehicleStatusMonitor()
#     try:
#         rclpy.spin(monitor_node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         monitor_node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
import time


class VehicleStatusMonitor(Node):
    def __init__(self):
        super().__init__('vehicle_status_monitor')

        # LaserScan Sensor Setup
        self.laserscan_subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Replace with your LaserScan topic
            self.laserscan_callback,
            10
        )
        self.laserscan_last_data_time = time.time()
        self.laserscan_status = "unknown"

        # IMU Sensor Setup
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',  # Replace with your IMU topic
            self.imu_callback,
            10
        )
        self.imu_last_data_time = time.time()
        self.imu_status = "unknown"

        # Publisher to publish combined status
        self.publisher_ = self.create_publisher(String, 'multi_sensor_status', 10)

        # Timer to check sensor status every second
        self.timer = self.create_timer(1.0, self.check_sensor_status)

    def laserscan_callback(self, msg):
        """Callback for LaserScan sensor."""
        self.get_logger().info('LaserScan data received.')
        self.laserscan_last_data_time = time.time()

    def imu_callback(self, msg):
        """Callback for IMU sensor."""
        self.get_logger().info('IMU data received.')
        self.imu_last_data_time = time.time()

    def check_sensor_status(self):
        """Check the status of both sensors."""
        current_time = time.time()

        # LaserScan Sensor Status Check
        if current_time - self.laserscan_last_data_time > 2.0:
            if self.laserscan_status != "malfunction":
                self.laserscan_status = "malfunction"
                self.log_and_report('LaserScan sensor: Malfunction')
        else:
            if self.laserscan_status != "operational":
                self.laserscan_status = "operational"
                self.log_and_report('LaserScan sensor: Operational')

        # IMU Sensor Status Check
        if current_time - self.imu_last_data_time > 2.0:
            if self.imu_status != "malfunction":
                self.imu_status = "malfunction"
                self.log_and_report('IMU sensor: Malfunction')
        else:
            if self.imu_status != "operational":
                self.imu_status = "operational"
                self.log_and_report('IMU sensor: Operational')

        # Publish Combined Status on Separate Lines
        status_message = String()
        status_message.data = (
            f"LaserScan: {self.laserscan_status}\nIMU: {self.imu_status}"
        )
        self.publisher_.publish(status_message)
        self.get_logger().info(f"Published:\n{status_message.data}")

    def log_and_report(self, message):
        """Log the message and optionally write it to a file."""
        if "Malfunction" in message:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

        # Uncomment if logging to a file is necessary:
        # with open('multi_sensor_status_report.txt', 'a') as report_file:
        #     report_file.write(f'{time.ctime()} - {message}\n')


def main(args=None):
    rclpy.init(args=args)
    monitor_node = VehicleStatusMonitor()
    try:
        rclpy.spin(monitor_node)
    except KeyboardInterrupt:
        pass
    finally:
        monitor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
