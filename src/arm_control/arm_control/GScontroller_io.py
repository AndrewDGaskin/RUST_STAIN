import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

class ControllerManager(Node):
    def __init__(self):
        super().__init__('contoller_input_subscriber')
        self.subscription = self.create_subscription(Joy,'joy', self.listener_output,10)
        self.publisher = self.create_publisher(Float32MultiArray, 'topic_controller_publisher', 10)

    def listener_output(self, msg):

        if len(msg.axes) < 4 or len(msg.buttons) < 2:
            return        

        #Subscribe to joystick input
        joint0 = msg.axes[0]                                            # Joystick axis for base joint
        if 0.05 < joint0 < 0.05:
            joint0 = 0.0
        joint1 = msg.axes[1]                                            # Joystick axis for shoulder joint
        if -0.05 < joint1 < 0.05:
            joint1 = 0.0
        joint2 = msg.axes[2]                                            # Joystick axis for elbow joint
        if -0.05 < joint2 < 0.05:
            joint2 = 0.0
        joint3 = msg.axes[3]                                            # Joystick axis for wrist joint
        if -0.05 < joint3 < 0.05: 
            joint3 = 0.0
        clawopen = msg.buttons[0]                                       # Joystick button for opening the claw
        clawclose = msg.buttons[1]                                      # Joystick button for closing the claw
        #print(f'GS receives controller input: Joint 0: {joint0}, Joint 1: {joint1}, Joint 2: {joint2}, Joint 3: {joint3}, Claw Open: {clawopen}, Claw Close: {clawclose}')

        #Publish the joint and claw values to the controller topic
        msg = Float32MultiArray() 
        msg.data = [joint0, joint1, joint2, joint3, clawopen, clawclose]
        self.publisher.publish(msg) 
        #print(f'GS sends: {msg.data}')
    

def main():
    rclpy.init()
    ROS_node_controller_manager = ControllerManager()
    rclpy.spin(ROS_node_controller_manager)

if __name__ == '__main__': 
    main() 