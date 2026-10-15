import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class PicoFeedback(Node):
    def __init__(self):
        super().__init__('pico_feedback')
        self.command_subscription = self.create_subscription(
            Float32MultiArray,
            'topic_controller_publisher',
            self.send_pico_command,
            10
        )
        PICO_PORT = '/dev/rover_pico'
        self.publisher = self.create_publisher(Float32MultiArray, 'topic_feedback', 10)
        self.pico = serial.Serial(PICO_PORT, 115200, timeout=0.1)
        self.timer = self.create_timer(0.05, self.read_pico_feedback)
        

    def read_pico_feedback(self):
        line = self.pico.readline().decode('utf-8').rstrip()

        if not line:
            return

        # Ignore anything from the Pico that is not telemetry
        if not line.startswith('TEL,'):
            print(f'Pico: {line}')
            return

        # Remove "TEL," and split the remaining telemetry values
        values = line.split(',')[1:]

        # Expecting 11 telemetry values
        if len(values) == 11:
            try:
                feedback_values = [float(value) for value in values]

                print(f'Feedback from Pico: {line}')

                feedback_msg = Float32MultiArray()
                feedback_msg.data = feedback_values
                self.publisher.publish(feedback_msg)

            except ValueError:
                self.get_logger().error(
                    f'Invalid feedback values: {values}'
                )

        else:
            self.get_logger().error(
                f'Unexpected number of feedback values: {values}'
            )

    def send_pico_command(self, msg):
        if len(msg.data) < 6:
            return

        command_string = ','.join(str(value) for value in msg.data[:6]) + '\n'
        self.pico.write(command_string.encode('utf-8'))
                
        
    

def main():
    rclpy.init()
    ROS_node_picofeedback = PicoFeedback()
    rclpy.spin(ROS_node_picofeedback)

if __name__ == '__main__': 
    main() 