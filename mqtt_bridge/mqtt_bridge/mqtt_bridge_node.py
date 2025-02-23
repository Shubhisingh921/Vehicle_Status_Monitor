import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt


class MQTTBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')

        # ROS Subscriber
        self.subscription = self.create_subscription(
            String,
            'multi_sensor_status',  # ROS topic to read from
            self.ros_callback,
            10
        )

        # MQTT Setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.connect('localhost', 1883, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f'Connected to MQTT Broker with result code {rc}')
        # Subscribe to the topic to receive messages (optional for localhost forwarding)
        client.subscribe('vehicle_status_topic')

    def on_message(self, client, userdata, msg):
        # Forward the MQTT message to localhost or process it locally
        self.get_logger().info(f"Forwarding MQTT to localhost: {msg.payload.decode()}")
        self.forward_to_localhost(msg.payload.decode())

    def ros_callback(self, msg):
        # Send ROS messages to the MQTT broker
        self.get_logger().info(f"Publishing ROS to MQTT: {msg.data}")
        self.mqtt_client.publish('vehicle_status_topic', msg.data)

    def forward_to_localhost(self, data):
        try:
            file_path = 'shubhi/logs/vehicle_status.log' 
            os.makedirs(os.path.dirname(file_path), exist_ok=True) 
            
            with open(file_path, 'a') as file:
                file.write(data + '\n')
        
            self.get_logger().info(f"Data written to {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write to {file_path}: {e}")


    def destroy_node(self):
        # Ensure proper cleanup
        self.mqtt_client.loop_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
