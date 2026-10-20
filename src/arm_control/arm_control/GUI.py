import sys

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QGridLayout,
    QVBoxLayout,
    QGroupBox
)


class GroundStationGUI(QWidget):

    def __init__(self):
        super().__init__()

        ##self.connection_status = QLabel("Disconnected")

        self.setWindowTitle("RUST STAIN Ground Station")

        self.axis0_value = QLabel("0.00")
        self.joint1_value = QLabel("0.00")
        self.joint2_value = QLabel("0.00")
        self.joint3_value = QLabel("0.00")
        self.claw_value = QLabel("0.00")

        self.accel_x_value = QLabel("0.00")
        self.accel_y_value = QLabel("0.00")
        self.accel_z_value = QLabel("0.00")

        self.temperature_value = QLabel("0.00")
        self.pressure_value = QLabel("0.00")
        self.altitude_value = QLabel("0.00")

        self.setup_layout()


    def setup_layout(self):

        ##self.connection_status.setStyleSheet("color: Red; font-weight: bold;")

        arm_layout = QGridLayout()

        arm_layout.addWidget(QLabel("Axis 0 Command:"), 0, 0)
        arm_layout.addWidget(self.axis0_value, 0, 1)

        arm_layout.addWidget(QLabel("Joint 1 Target:"), 1, 0)
        arm_layout.addWidget(self.joint1_value, 1, 1)

        arm_layout.addWidget(QLabel("Joint 2 Target:"), 2, 0)
        arm_layout.addWidget(self.joint2_value, 2, 1)

        arm_layout.addWidget(QLabel("Joint 3 Target:"), 3, 0)
        arm_layout.addWidget(self.joint3_value, 3, 1)

        arm_layout.addWidget(QLabel("Claw Target:"), 4, 0)
        arm_layout.addWidget(self.claw_value, 4, 1)

        arm_group = QGroupBox("Arm")
        arm_group.setLayout(arm_layout)


        sensor_layout = QGridLayout()

        sensor_layout.addWidget(QLabel("Accel X:"), 0, 0)
        sensor_layout.addWidget(self.accel_x_value, 0, 1)

        sensor_layout.addWidget(QLabel("Accel Y:"), 1, 0)
        sensor_layout.addWidget(self.accel_y_value, 1, 1)

        sensor_layout.addWidget(QLabel("Accel Z:"), 2, 0)
        sensor_layout.addWidget(self.accel_z_value, 2, 1)

        sensor_layout.addWidget(QLabel("Temperature:"), 3, 0)
        sensor_layout.addWidget(self.temperature_value, 3, 1)

        sensor_layout.addWidget(QLabel("Pressure:"), 4, 0)
        sensor_layout.addWidget(self.pressure_value, 4, 1)

        sensor_layout.addWidget(QLabel("Altitude:"), 5, 0)
        sensor_layout.addWidget(self.altitude_value, 5, 1)

        sensor_group = QGroupBox("Sensors")
        sensor_group.setLayout(sensor_layout)


        main_layout = QVBoxLayout()

        main_layout.addWidget(arm_group)
        main_layout.addWidget(sensor_group)
        ##main_layout.addWidget(self.connection_status)

        self.setLayout(main_layout)


    def update_feedback(self, data):

        if len(data) != 11:
            return

        self.axis0_value.setText(f"{data[0]:.2f}")

        self.joint1_value.setText(f"{data[1]:.2f}°")
        self.joint2_value.setText(f"{data[2]:.2f}°")
        self.joint3_value.setText(f"{data[3]:.2f}°")
        self.claw_value.setText(f"{data[4]:.2f}°")

        self.accel_x_value.setText(f"{data[5]:.2f} m/s²")
        self.accel_y_value.setText(f"{data[6]:.2f} m/s²")
        self.accel_z_value.setText(f"{data[7]:.2f} m/s²")

        self.temperature_value.setText(f"{data[8]:.2f} °C")
        self.pressure_value.setText(f"{data[9]:.2f} Pa")
        self.altitude_value.setText(f"{data[10]:.2f} m")

    def check_connection(self):
        


def main():

    app = QApplication(sys.argv)

    window = GroundStationGUI()

    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()