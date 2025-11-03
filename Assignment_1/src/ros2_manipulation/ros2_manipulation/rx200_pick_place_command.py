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
        msg = Int32()
        msg.data = self.state
        self.pub_command.publish(msg)
        self.get_logger().info(f'Sent state {self.state}')
        self.state += 1
        if self.state == 9: 
            self.get_logger().info("PICK-AND-PLACE COMPLETE")
            #need to work on this yet, not working right now
            self.destroy_node()  # Destroy the node to prevent further execution
            rclpy.shutdown()
    
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