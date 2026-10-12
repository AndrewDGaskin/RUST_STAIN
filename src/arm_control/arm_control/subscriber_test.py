import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberTest(Node):
    def __init__(self):
        super().__init__('subscriber_test')
        self.subscription = self.create_subscription(String,'test_topic',self.listener_output,10)

    def listener_output(self, msg):
        print(f'Received: {msg.data}')


def main():
    rclpy.init()
    ROS_node = SubscriberTest()
    rclpy.spin(ROS_node)

if __name__ == '__main__': 
    main() 