import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import struct
import time


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # IMU configuration
        self.port = '/dev/ttyUSB0'  # Adjust the port as needed
        self.baud_rate = 9600
        self.last_data_received_time = time.time()  # Time of last valid data
        self.malfunction_threshold = 2.0  # Seconds to declare a malfunction

        # Attempt to connect to the IMU
        try:
            self.serial_connection = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Connected to IMU on {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to IMU: {e}')
            self.serial_connection = None

        # Create publisher and timers
        self.publisher = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.poll_timer = self.create_timer(0.1, self.poll_imu)  # Poll IMU every 0.1 seconds
        self.status_timer = self.create_timer(1.0, self.check_sensor_status)  # Check IMU status every second

    def poll_imu(self):
        if not self.serial_connection:
            return

        try:
            # Read raw binary data from IMU
            line = self.serial_connection.readline()

            # Parse the binary data (adjust parsing based on your IMU's format)
            imu_data = self.parse_imu_data(line)
            if imu_data:
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = imu_data

                # Create and publish the IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'imu_link'

                imu_msg.linear_acceleration.x = accel_x
                imu_msg.linear_acceleration.y = accel_y
                imu_msg.linear_acceleration.z = accel_z
                imu_msg.angular_velocity.x = gyro_x
                imu_msg.angular_velocity.y = gyro_y
                imu_msg.angular_velocity.z = gyro_z

                self.publisher.publish(imu_msg)
                self.get_logger().info(f'Published IMU data: {imu_data}')

                # Update the last data received time
                self.last_data_received_time = time.time()

        except struct.error as e:
            self.get_logger().error(f'Error parsing IMU data: {e}')
        except serial.SerialException as e:
            self.get_logger().error(f'Error reading from IMU: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')

    def parse_imu_data(self, line):
        """
        Parse binary data received from the IMU.
        Adjust this function based on your IMU's specific data format.
        """
        if len(line) >= 24:  # Ensure sufficient data is received
            try:
                # Unpack 6 floats (4 bytes each) from the binary data
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = struct.unpack('<ffffff', line[:24])
                return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
            except struct.error as e:
                self.get_logger().error(f'Failed to unpack data: {e}')
        else:
            self.get_logger().warn(f'Incomplete data received: {line}')
        return None

    def check_sensor_status(self):
        # Check if data has been received recently
        if time.time() - self.last_data_received_time > self.malfunction_threshold:
            self.get_logger().error('IMU malfunction: No data received')
        else:
            self.get_logger().info('IMU sensor is working fine')

    def destroy_node(self):
        # Ensure the serial connection is closed on shutdown
        if self.serial_connection:
            self.serial_connection.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
