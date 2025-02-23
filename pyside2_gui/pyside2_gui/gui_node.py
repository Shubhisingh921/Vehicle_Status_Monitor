import sys
from PySide2.QtWidgets import (
    QApplication,
    QMainWindow,
    QVBoxLayout,
    QWidget,
    QTextEdit,
    QPushButton,
    QLabel,
    QHBoxLayout,
    QFrame,
)
from PySide2.QtCore import QTimer, Qt
import paho.mqtt.client as mqtt


class MQTTGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()

        # MQTT client setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect("localhost", 1883, 60)
        self.mqtt_client.loop_start()

        # Timer for real-time updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_logs)
        self.timer.start(500)  # Update every 500ms

        self.logs = []
        self.connection_status = "Disconnected"
        self.last_status = None  # Track the last status (e.g., 'Operational', 'Malfunction')

    def init_ui(self):
        self.setWindowTitle("Vehicle Status Monitor")
        self.setGeometry(200, 200, 800, 600)

        # Main layout
        main_layout = QVBoxLayout()

        # Header and Connection Status
        header_layout = QHBoxLayout()

        header = QLabel("Vehicle Status Monitor")
        header.setStyleSheet("font-size: 20px; font-weight: bold; color: #007BFF;")
        header.setAlignment(Qt.AlignLeft)
        header_layout.addWidget(header)

        self.status_label = QLabel("MQTT: Disconnected")
        self.status_label.setStyleSheet("font-size: 14px; color: #FF0000;")
        self.status_label.setAlignment(Qt.AlignRight)
        header_layout.addWidget(self.status_label)

        main_layout.addLayout(header_layout)

        # Separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        main_layout.addWidget(separator)

        # Log viewer
        self.log_viewer = QTextEdit()
        self.log_viewer.setReadOnly(True)
        self.log_viewer.setStyleSheet(
            """
            font-size: 14px;
            background-color: #F9F9F9;
            border: 1px solid #CCCCCC;
            padding: 8px;
        """
        )
        main_layout.addWidget(self.log_viewer, stretch=1)

        # Buttons layout
        button_layout = QHBoxLayout()

        self.clear_button = QPushButton("Clear Logs")
        self.clear_button.clicked.connect(self.clear_logs)
        self.clear_button.setStyleSheet(
            """
            QPushButton {
                font-size: 14px;
                background-color: #F0F0F0;
                border: 1px solid #CCCCCC;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: #E0E0E0;
            }
        """
        )
        button_layout.addWidget(self.clear_button)

        self.exit_button = QPushButton("Exit")
        self.exit_button.clicked.connect(self.close_application)
        self.exit_button.setStyleSheet(
            """
            QPushButton {
                font-size: 14px;
                background-color: #FF4C4C;
                color: white;
                border: 1px solid #FF0000;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: #FF6666;
            }
        """
        )
        button_layout.addWidget(self.exit_button)

        main_layout.addLayout(button_layout)

        # Central widget
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def on_connect(self, client, userdata, flags, rc):
        self.logs.append("<b>Connected to MQTT Broker</b>")
        self.connection_status = "Connected"
        self.update_connection_status()
        client.subscribe("vehicle_status_topic")


    def on_message(self, client, userdata, msg):
        message = msg.payload.decode()
        
        if "malfunction" in message.lower() and "operational" in message.lower():
            if self.last_status != "malfunction":
                self.logs.append(f"<span style='color:red;'><b>Error: {message}</b></span>")
                self.last_status = "malfunction"
                
        elif "malfunction" in message.lower() and self.last_status != "malfunction":
            self.logs.append(f"<span style='color:red;'><b>Error: {message}</b></span>")
            self.last_status = "malfunction"
        elif "operational" in message.lower() and self.last_status != "operational":
            self.logs.append(f"<span style='color:green;'>Info: {message}</span>")
            self.last_status = "operational"
    # Skip logging if the status hasn't changed



    def update_logs(self):
        if self.logs:
            for log in self.logs:
                self.log_viewer.append(log)
            self.logs.clear()
            # Auto-scroll to the bottom
            self.log_viewer.verticalScrollBar().setValue(
                self.log_viewer.verticalScrollBar().maximum()
            )

    def update_connection_status(self):
        color = "#007BFF" if self.connection_status == "Connected" else "#FF0000"
        self.status_label.setText(f"MQTT: {self.connection_status}")
        self.status_label.setStyleSheet(f"font-size: 14px; color: {color};")

    def clear_logs(self):
        self.log_viewer.clear()

    def close_application(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        self.close()


def main():
    app = QApplication(sys.argv)
    gui = MQTTGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
