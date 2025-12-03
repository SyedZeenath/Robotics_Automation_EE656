#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import json
import tf2_ros
import numpy as np

from interbotix_perception_msgs.srv import ClusterInfoArray
from tf_transformations import euler_matrix

class PickPlacePerception(Node):
    def __init__(self):
        super().__init__('rx200_pick_place_perception')
        self.base_link = 'rx200/base_link'
        self.ref_frame = 'camera_color_optical_frame'
        
        self.pub = self.create_publisher(String, '/detected_blocks', 10)
        
        self.srv_get_cluster_positions = self.create_client(
            ClusterInfoArray,
            f'/{filter_ns}/get_cluster_positions'
        )
        while not self.srv_get_cluster_positions.wait_for_service(1.0):
            self.get_logger().info(
                f"Waiting for services '{self.srv_get_cluster_positions.srv_name}', come up.")
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(f"Perception Node Initialised")        

    def get_cluster_positions(self) -> Tuple[bool, Union[None, List[dict]]]:
        """
        Get the estimated positions of all pointcloud clusters.
        """

        clusters = self.srv_get_cluster_positions.call_async(
            ClusterInfoArray.Request()
        )
        self.node_inf.wait_until_future_complete(future_get_cluster_pos)
        clusters = future_get_cluster_pos.result().clusters
        if len(clusters) == 0:
            self.get_logger().warning('No clusters found...')
            return False, []
        
        # Get the cluster frame from the first cluster
        cluster_frame = clusters[0].frame_id
        # Get the transform from the 'ref_frame' to the cluster frame (i.e. the camera's depth
        # frame) - known as T_rc
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame=self.ref_frame,
                source_frame=cluster_frame,
                time=Time(),
                timeout=Duration(seconds=4.0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):
            self.node_inf.logerror(
                f"Failed to look up the transform from '{ref_frame}' to '{cluster_frame}'."
            )
            return False, []
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        quat = trans.transform.rotation
        rpy = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        T_rc = self.pose_to_matrix([x, y, z, rpy[0], rpy[1], rpy[2]])

        # Transform the clusters to be w.r.t. the 'ref_frame' instead of the camera's depth frame
        transformed_clusters = []
        for idx, cluster in clusters:
            # p_co is the cluster's position w.r.t. the camera's depth frame
            # p_ro is the cluster's position w.r.t. the desired reference frame
            p_co = [cluster.position.x, cluster.position.y, cluster.position.z, 1]
            p_ro = np.dot(T_rc, p_co)

            transformed_clusters.append({
                'name': f'cluster_{idx}',
                'position': [p_ro[0], p_ro[1], p_ro[2]],
                'color': [cluster.color.r, cluster.color.g, cluster.color.b],
                'num_points': cluster.num_points
            })

        return True, transformed_clusters

    def pose_to_matrix(self, pose: List[float]) -> np.ndarray:
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