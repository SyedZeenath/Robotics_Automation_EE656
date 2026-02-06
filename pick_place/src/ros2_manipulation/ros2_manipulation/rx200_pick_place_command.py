#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class PickPlaceCommand(Node):
    def __init__(self):
        super().__init__('rx200_pick_place_command')
        
        self.pub_command = self.create_publisher(Int32, '/pick_place_command', 10)
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.state = 0 # state is the command here
        self.get_logger().info('Timer node is initialized')
        
    def timer_callback(self):
        '''
        This method triggers every 5seconds. It passes the state/position of the arm.
        The state is currently fixed to be 8 states the arm moves from pick point to place point, including the open 
        and close of the gripper. So, we end the sequence once the state hits 9. 
        '''
        msg = Int32()
        msg.data = self.state
        self.pub_command.publish(msg)
        self.get_logger().info(f'Sent state {self.state}')
        self.state += 1
        if self.state == 9: 
            self.get_logger().info("PICK-AND-PLACE COMPLETE")
            self.state = 0

    
def main():
    rclpy.init()
    node = PickPlaceCommand()   
    try:
        rclpy.spin(node)        
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()