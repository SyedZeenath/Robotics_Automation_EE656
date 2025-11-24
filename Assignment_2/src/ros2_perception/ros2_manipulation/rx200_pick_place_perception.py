#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import json

from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

class PickPlacePerception(Node):
    def __init__(self):
        super().__init__('rx200_pick_place_perception')
        self.base_link = 'rx200/base_link'
        REF_FRAME = 'camera_color_optical_frame'
        ARM_TAG_FRAME = 'camera_color_optical_frame'  # Since camera is on EE
        
        self.pub = self.create_publisher(String, '/detected_blocks', 10)
        
        # Getting armTag and pointcloud interfaces - TO DO: to use without AprilTag
        self.pcl = InterbotixPointCloudInterface(
            node_inf=self,
        )

        # Since camera is on end-effector, and the static transform is already publsihed
        # this interface might not be required.
        
        #TODO: Test it out
        
        # self.armtag = InterbotixArmTagInterface(
        #     ref_frame=REF_FRAME,
        #     arm_tag_frame=ARM_TAG_FRAME,
        #     arm_base_frame=self.base_link,
        #     node_inf=self,
        # )
        
        self.timer = self.create_timer(2.0, self.detect_blocks)
        self.get_logger().info('Perception node is initialized')
    
    def extract_color_names(self, cluster_color):
        """
        Classify block color based on RGB values.
        Returns: "red", "blue", "yellow", or "unknown"
        """
        r, g, b = cluster_color
        if r > 150 and g < 100 and b < 100:
            return "red"
        if b > 150 and r < 100 and g < 100:
            return "blue"
        if r > 150 and g > 150 and b < 100:
            return "yellow"
        return "unknown" 
      
    def detect_blocks(self):
        # Get the arm pose 
        # self.armtag.find_ref_to_arm_base_transform()
        
        # get the pointcloud clusters to detect blocks
        clusters = self.pcl.get_cluster_positions(
            ref_frame=self.base_link,
            sort_axis='x',
            reverse=True
        )        
        # Create a dictionary to hold detected block positions by color
        detected_blocks = {}
        for cluster in clusters:
            cluster_color = self.extract_color_names(cluster['color'])
            cluster_position = cluster['position']
            detected_blocks[cluster_color] = cluster_position

        msg = String()
        msg.data = json.dumps(detected_blocks)
        self.pub.publish(msg)
        self.get_logger().info(f'Published detected blocks: {msg.data}')

            
def main():
    rclpy.init()
    node = PickPlacePerception()   
    try:
        rclpy.spin(node)        
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()