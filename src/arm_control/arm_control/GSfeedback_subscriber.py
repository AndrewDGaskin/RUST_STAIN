import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import csv
from datetime import datetime

class FeedbackSubscriber(Node):
    def __init__(self):
        super().__init__('feedback_subscriber')
        self.subscription = self.create_subscription(Float32MultiArray, 'topic_feedback', self.read_op_feedback, 10)
        self.csv_file = open('telemetry.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            'timestamp',
            'joint0_motor_command',
            'joint1_target',
            'joint2_target',
            'joint3_target',
            'claw_target',
            'accel_x_m_s2',
            'accel_y_m_s2',
            'accel_z_m_s2',
            'temperature_c',
            'pressure_pa',
            'altitude_m'
        ])

    def read_op_feedback(self, msg):
        print(f'GS Received: {msg.data}')

        if len(msg.data) != 11:
            return

        # Arm telemetry
        joint0 = msg.data[0]
        joint1 = msg.data[1]
        joint2 = msg.data[2]
        joint3 = msg.data[3]
        claw = msg.data[4]

        # BNO055 telemetry
        accel_x = msg.data[5]
        accel_y = msg.data[6]
        accel_z = msg.data[7]

        # BMP388 telemetry
        temperature = msg.data[8]
        pressure = msg.data[9]
        altitude = msg.data[10]

        self.csv_writer.writerow([
            datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            joint0,
            joint1,
            joint2,
            joint3,
            claw,
            accel_x,
            accel_y,
            accel_z,
            temperature,
            pressure,
            altitude
        ])

        self.csv_file.flush()


def main():
    rclpy.init()
    ROS_node_feedback_subscriber = FeedbackSubscriber()
    rclpy.spin(ROS_node_feedback_subscriber)

if __name__ == '__main__': 
    main() 