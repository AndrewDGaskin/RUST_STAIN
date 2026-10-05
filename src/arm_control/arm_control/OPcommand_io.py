import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('command_subscriber')
        self.subscription = self.create_subscription(Float32MultiArray,'topic_controller_publisher', self.listener_output, 10)

    def listener_output(self, msg):

        
        #print(f'OP Received: {msg.data}')
        if len(msg.data) < 6:
            return

        
        #Extract joint and claw values from the received message
        joint0 = msg.data[0]
        joint1 = msg.data[1]
        joint2 = msg.data[2]
        joint3 = msg.data[3]
        clawopen = msg.data[4]
        clawclose = msg.data[5]

        #Prep to transmit the command values to Pico
        command_values = [joint0, joint1, joint2, joint3, clawopen, clawclose]
        command_string = ','.join(map(str, command_values))
        print(f'Command to send to Pico: {command_string}')


def main():
    rclpy.init()
    ROS_node = CommandSubscriber()
    rclpy.spin(ROS_node)

if __name__ == '__main__': 
    main() 