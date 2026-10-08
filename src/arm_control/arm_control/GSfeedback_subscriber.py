import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class FeedbackSubscriber(Node):
    def __init__(self):
        super().__init__('feedback_subscriber')
        self.subscription = self.create_subscription(Float32MultiArray, 'topic_feedback', self.read_op_feedback, 10)

    def read_op_feedback(self, msg):
        #print(f'OP Received: {msg.data}')
        if len(msg.data) < 9:
            return

        #Extract joint and claw values from the received message
        joint0 = msg.data[0]
        joint1 = msg.data[1]
        joint2 = msg.data[2]
        joint3 = msg.data[3]
        clawopen = msg.data[4]
        clawclose = msg.data[5]
        temp = msg.data[6]
        pressure = msg.data[7]
        acceleration = msg.data[8]


def main():
    rclpy.init()
    ROS_node_feedback_subscriber = FeedbackSubscriber()
    rclpy.spin(ROS_node_feedback_subscriber)

if __name__ == '__main__': 
    main() 