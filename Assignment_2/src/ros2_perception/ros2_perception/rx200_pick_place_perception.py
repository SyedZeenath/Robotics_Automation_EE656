#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import json
import tf2_ros
import numpy as np

from interbotix_perception_msgs.srv import ClusterInfoArray
from tf_transformations import euler_matrix, euler_from_quaternion

from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_geometry_msgs
from rcl_interfaces.msg import SetParametersResult

class PickPlacePerception(Node):
    def __init__(self):
        super().__init__('rx200_pick_place_perception')
        self.base_link = 'rx200/base_link'
        self.ref_frame = 'camera_link'
        self.wrist_link = 'rx200/wrist_link'
        
        self.pub = self.create_publisher(String, '/detected_blocks', 10)
        
        self.declare_parameter('pick_order', value=['red', 'blue', 'yellow'])
        self._pick_order = self.get_parameter('pick_order').value
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.srv_get_cluster_positions = self.create_client(
            ClusterInfoArray,
            f'/pc_filter/get_cluster_positions'
        )
        while not self.srv_get_cluster_positions.wait_for_service(1.0):
            self.get_logger().info(
                f"Waiting for services '{self.srv_get_cluster_positions.srv_name}', come up.")
        self.clusterArray = ClusterInfoArray.Request()

        self.br = tf2_ros.TransformBroadcaster(self)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(f"Perception Node Initialised!")  
        # self.timer = self.create_timer(5.0, self.detect_blocks)  
        self.detect_blocks()    

    def parameter_callback(self, params):
        """
        Called automatically whenever a parameter is updated.
        params: list of Parameter objects
        """
        for param in params:
            if param.name == 'pick_order':
                if isinstance(param.value, list) and all(isinstance(i, str) for i in param.value):
                    self.get_logger().info(f"pick_order updated to {param.value}")
                    self._pick_order = param.value
                    self.detect_blocks()
                else:
                    self.get_logger().warn("pick_order must be a list of strings")
        return SetParametersResult(successful=True)
    
    def get_cluster_positions(self):
        """
        Get the estimated positions of all pointcloud clusters.
        """

        clusters = self.srv_get_cluster_positions.call_async(self.clusterArray)
        rclpy.spin_until_future_complete(self, clusters)
        clusters = clusters.result().clusters
        # clusters = [
        #     {
        #         'frame_id': 'camera_depth_optical_frame',
        #         'name': 'cluster_1', 
        #         'position': [-0.021947862580418587, -0.12682467699050903, 0.48142504692077637], 
        #         'yaw': 0, 
        #         'color': [121.0, 107.0, 88.0], 
        #         'num_points': 666
        #      }, 
        #     {
        #         'frame_id': 'camera_depth_optical_frame',
        #         'name': 'cluster_2', 
        #         'position': [-0.0340929739177227, -0.12213777750730515, 0.4699997901916504], 
        #         'yaw': 0, 
        #         'color': [203.0, 49.0, 56.0], 
        #         'num_points': 311
        #         }
        #     ]
        num_clusters = len(clusters)
        if num_clusters == 0:
            self.get_logger().warning('No clusters found...')
            return False, []

        # Get the cluster frame from the first cluster
        cluster_frame = clusters[0]['frame_id']
        self.get_logger().info(f"Looking up transform from '{self.base_link}' to '{cluster_frame}'")
        
        # --- Look up transform: base_link ← cluster_frame ---
        try:
            tf_cam_to_base = self.tf_buffer.lookup_transform(
                target_frame=self.base_link,
                source_frame=cluster_frame,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            self.get_logger().info(f"********Transform lookup successful! ********")
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):
            self.get_logger().error(
                f"Failed to look up the transform from '{self.wrist_link}' to '{cluster_frame}'."
            )
            return []
        
        # --- Transform each cluster pose into base_link ---  
        transformed_clusters = []
        tf2_converter = tf2_geometry_msgs

        final_trans = []
        time_now = self.get_clock().now().to_msg()

        cluster_id = 1

        for cluster in clusters:
            # Construct PoseStamped in camera frame
            pose_in_cam = PoseStamped()
            pose_in_cam.header.frame_id = cluster_frame
            pose_in_cam.pose.position = cluster.position
            pose_in_cam.pose.orientation.w = 1.0  # no orientation from clustering

            # Transform using TF2
            pose_in_base = tf2_geometry_msgs.do_transform_pose(pose_in_cam, tf_cam_to_base)

            # Save transformed coordinates
            x = pose_in_base.pose.position.x
            y = pose_in_base.pose.position.y
            z = pose_in_base.pose.position.z

            trans = TransformStamped()
            trans.header.frame_id = self.base_link
            trans.header.stamp = time_now
            trans.child_frame_id = f'cluster_{str(cluster_num)}'
            trans.transform.translation.x = x
            trans.transform.translation.y = y
            trans.transform.translation.z = z
            trans.transform.rotation = Quaternion(x=0., y=0., z=0., w=1.)
            final_trans.append(trans)

                # Add to final cluster list
            transformed_clusters.append({
                'name': trans.child_frame_id,
                'position': [x, y, z],
                'yaw': 0,
                'color': [cluster.color.r, cluster.color.g, cluster.color.b],
                'num_points': cluster.num_points
            })
            
            cluster_id += 1
            
        self.br.sendTransform(final_trans)
        
        return final_clusters

    def pose_to_matrix(self, pose):
        mat = np.identity(4)
        theta = pose[3:]
        mat[:3, :3] = euler_matrix(theta[0], theta[1], theta[2], axes='sxyz')[:3, :3]
        mat[:3, 3] = pose[:3]
        return mat

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
        # get the pointcloud clusters to detect blocks
        clusters = self.get_cluster_positions()        
        self.get_logger().info(f"Number of clusters detected: {len(clusters)}")
        # Create a dictionary to hold detected block positions by color
        
        # TO work without robot and camera, hardcoding clusters        
        # clusters = [
        #     {
        #         'name': 'cluster_1',
        #         'position': [0.3, 0.2, 0.2],
        #         'yaw': 0,
        #         'color': [203.0, 49.0, 56.0],
        #         'num_points': 311
        #     },
        #     {
        #         'name': 'cluster_2',
        #         'position': [0.2, 0.2, 0.1],
        #         'yaw': 0,
        #         'color': [255.0, 255.0, 0.0],
        #         'num_points': 250
        #     },
        #     {
        #         'name': 'cluster_3',
        #         'position': [0.25, 0.3, 0.1],
        #         'yaw': 0,
        #         'color': [49.0, 66.0, 255.0],
        #         'num_points': 200
        #     }
        # ]
        
        detected_blocks = {
            'color': {},
            'pick_order': []
        }
        for cluster in clusters:
            cluster_color = self.extract_color_names(cluster['color'])
            if( cluster_color == "unknown"):
                continue
            cluster_position = cluster['position']
            detected_blocks['color'][cluster_color] = cluster_position
            detected_blocks['pick_order'] = self._pick_order
        
        # detected_blocks = {
        #                    "red": [0.3, -0.2, 0.1],
        #                    "yellow": [0.2, 0.2, 0.1],
        #                    "blue": [0.25, 0.3, 0.1]
        #                    }

        msg = String()
        msg.data = json.dumps(detected_blocks)
        self.pub.publish(msg)
        self.get_logger().info(f'Published detected blocks: {msg}')

            
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



#THIS IS JUST FOR UNDERSTANDING PURPOSES
#  response:


# Clusters: '[interbotix_perception_msgs.msg.ClusterInfo(frame_id='camera_depth_optical_frame', position=geometry_msgs.msg.Point(x=0.045460764318704605, y=-0.10489796102046967, z=0.47484132647514343), yaw=0.0, color=std_msgs.msg.ColorRGBA(r=225.0, g=177.0, b=101.0, a=0.0), min_z_point=geometry_msgs.msg.Point(x=0.029922788962721825, y=-0.10594867914915085, z=0.46627557277679443), num_points=259), interbotix_perception_msgs.msg.ClusterInfo(frame_id='camera_depth_optical_frame', position=geometry_msgs.msg.Point(x=-0.021947862580418587, y=-0.12682467699050903, z=0.48142504692077637), yaw=0.0, color=std_msgs.msg.ColorRGBA(r=192.0, g=38.0, b=42.0, a=0.0), min_z_point=geometry_msgs.msg.Point(x=-0.0340929739177227, y=-0.12213777750730515, z=0.4699997901916504), num_points=196)]'


# data='{"red": [0.4698081011063667, 0.01762461051054736, 0.24748738600375847]}'