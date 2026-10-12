import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherTest(Node): 
    def __init__(self): 
        super().__init__('publisher_test') 
        self.publisher = self.create_publisher(String, 'test_topic', 10)
        self.counter = 0 
        self.timer = self.create_timer(1.0, self.publish_message) 

    def publish_message(self): 
        msg = String() 
        msg.data = f'Hello World {self.counter}' 
        self.publisher.publish(msg) 
        self.counter += 1
        print(f'Published: {msg.data}')

def main(): 
    rclpy.init() 
    ROS_node = PublisherTest() 
    rclpy.spin(ROS_node) 

if __name__ == '__main__': 
    main() 