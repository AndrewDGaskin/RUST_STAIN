import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class PicoFeedback(Node):
    def __init__(self):
        super().__init__('pico_feedback')
        self.publisher = self.create_publisher(Float32MultiArray, 'topic_feedback', 10)
        self.pico = serial.Serial('/dev/ttyACM*', 115200, timeout=0.1)
        self.timer = self.create_timer(0.05, self.read_pico_feedback)
        

    def read_pico_feedback(self):
        line = self.pico.readline().decode('utf-8').rstrip()
        if line:
            values = line.split(',')
            if len(values) == 5:  # Expecting 5 values: joint0_target, joint1_target, joint2_target, joint3_target, claw_target
                try:

                    #get values from pico, see if its corrupted, if not publish to feedback topic
                    feedback_values = [float(value) for value in values]
                    print(f'Feedback from Pico: {line}')
                    feedback_msg = Float32MultiArray()
                    feedback_msg.data = feedback_values
                    self.publisher.publish(feedback_msg)

                    #if corrupted, log error
                except ValueError:
                    self.get_logger().error(f'Invalid feedback values: {values}')
            else:
                self.get_logger().error(f'Unexpected number of feedback values: {values}')
                
        
    

def main():
    rclpy.init()
    ROS_node_picofeedback = PicoFeedback()
    rclpy.spin(ROS_node_picofeedback)

if __name__ == '__main__': 
    main() 