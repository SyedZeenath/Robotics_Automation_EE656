#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import json
import tf2_ros
import time

from interbotix_perception_msgs.srv import ClusterInfoArray
from tf_transformations import euler_from_quaternion

from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import Quaternion, TransformStamped
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from interbotix_common_modules import angle_manipulation as ang

class PickPlacePerception(Node):
    def __init__(self):
        super().__init__('rx200_pick_place_perception')
        self.base_link = 'rx200/base_link'
        self.ref_frame = 'camera_link'
        self.wrist_link = 'rx200/wrist_link'
        
        # ------------------
        # Publisher to send detected blocks
        # ------------------
        self.pub = self.create_publisher(String, '/detected_blocks', 10)
        
        # ------------------
        # Parameters passed from launch file/command line
        # ------------------
        self.declare_parameter('pick_order', value=['red', 'blue', 'yellow'])
        self._pick_order = self.get_parameter('pick_order').value
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # ------------------
        # Service Clients
        # ------------------
        self.srv_get_cluster_positions = self.create_client(ClusterInfoArray, '/pc_filter/get_cluster_positions')
        while not self.srv_get_cluster_positions.wait_for_service(1.0):
            self.get_logger().info(
                f"Waiting for service '{self.srv_get_cluster_positions.srv_name}', to come up.")
        self.clusterArray = ClusterInfoArray.Request()

        # ------------------
        # TF2 listener and broadcaster
        # ------------------
        self.br = tf2_ros.TransformBroadcaster(self)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ------------------
        # Detect blocks
        # ------------------
        # self.timer = self.create_timer(5.0, self.detect_blocks)  
        self.completed = False
        self.found_colors = set()
        self.detect_blocks()    
        
        self.get_logger().info(f"Perception Node Initialised!")  

    def parameter_callback(self, params):
        """
        Called automatically whenever pick_order is updated.
        params: list of Parameter objects
        """
        for param in params:
            if param.name == 'pick_order':
                if isinstance(param.value, list) and all(isinstance(i, str) for i in param.value):
                    self.get_logger().info(f"pick_order updated to {param.value}")
                    self._pick_order = param.value
                    self.completed = False
                    # self.timer.cancel()  # Cancel existing timer
                    # self.timer = self.create_timer(5.0, self.detect_blocks)  # Restart timer
                    self.detect_blocks()
                else:
                    self.get_logger().warn("pick_order must be a list of strings")
        return SetParametersResult(successful=True)
    
    def get_cluster_positions(self):
        """
        Get the estimated positions of all pointcloud clusters and transform into base frame.
        """
        clusters = self.srv_get_cluster_positions.call_async(self.clusterArray)
        rclpy.spin_until_future_complete(self, clusters)
        clusters = clusters.result().clusters
        num_clusters = len(clusters)
        
        if num_clusters == 0:
            self.get_logger().warning('No clusters found...')
            return []
        self.get_logger().info(f'Clusters ===== {clusters} ') 
          
        # Get the cluster frame id from the first cluster
        cluster_frame = clusters[0].frame_id
        
        # --- Look up transform: base_link ← cluster_frame ---
        self.get_logger().info(f"Looking up transform from '{self.base_link}' to '{cluster_frame}'")
        trans = self.safe_lookup_transform(self.base_link, cluster_frame)
        self.get_logger().info(f"********Transform lookup successful! ********")
        if trans is None:
            return []      
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        quat = trans.transform.rotation
        rpy = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        T_rc = ang.pose_to_transformation_matrix([x, y, z, rpy[0], rpy[1], rpy[2]])

        final_trans = []
        cluster_num = 1
        time_now = self.get_clock().now().to_msg()
        final_clusters = []
        
        for cluster in clusters:
            # pos_cam is the cluster's position w.r.t. the camera's depth frame
            # pos_base is the cluster's position w.r.t. the desired base frame
            pos_cam = [cluster.position.x, cluster.position.y, cluster.position.z, 1]
            pos_base = T_rc @ pos_cam
            
            cluster.position.x = pos_base[0]
            cluster.position.y = pos_base[1]
            cluster.position.z = pos_base[2]
        
        # publish transforms to the /tf tree for debugging purposes (only once)
        # for cluster in clusters:
            trans = TransformStamped()
            trans.header.frame_id = self.base_link
            trans.header.stamp = time_now
            trans.child_frame_id = f'cluster_{str(cluster_num)}'
            trans.transform.translation.x = cluster.position.x
            trans.transform.translation.y = cluster.position.y
            trans.transform.translation.z = cluster.position.z
            trans.transform.rotation = Quaternion(x=0., y=0., z=0., w=1.)
            # final_trans.append(trans)
            cluster_num += 1
            self.br.sendTransform(trans)

        # create a list of Python dictionaries to return to the user
        # for indx in range(num_clusters):
            # name = final_trans[indx].child_frame_id
            # x = final_trans[indx].transform.translation.x
            # y = final_trans[indx].transform.translation.y
            # z = final_trans[indx].transform.translation.z
            # yaw = 0
            # r = clusters[indx].color.r
            # g = clusters[indx].color.g
            # b = clusters[indx].color.b
            # num_points = clusters[indx].num_points
            cluster = {
                'name': f"cluster_{cluster_num}",
                'position': [cluster.position.x, cluster.position.y, cluster.position.z],
                'yaw': 0,
                'color': [cluster.color.r, cluster.color.g, cluster.color.b],
                'num_points': cluster.num_points
            }
            final_clusters.append(cluster)
        return final_clusters


    def safe_lookup_transform(self, target, source, attempts=10, delay=5.0):
        for i in range(attempts):
            try:
                return self.tf_buffer.lookup_transform(
                    target,
                    source,
                    Time(),
                    timeout=Duration(seconds=0.3)
                )
            except Exception as e:
                if i == attempts - 1:
                    self.get_logger().error(
                        f"TF lookup failed after {attempts} attempts: {e}"
                    )
                    return None

                self.get_logger().warn(
                    f"TF lookup failed (attempt {i+1}/{attempts}). Retrying..."
                )
                time.sleep(delay)

    def extract_color_names(self, cluster_color):
        """
        Classify block color based on RGB values.
        Returns: "red", "blue", "yellow", or "unknown"
        """
        r, g, b = cluster_color
        if r > 150 and g < 120 and b < 120:
            return "red"
        if b > max(r, g) + 20 and b < 120 and r < 100 and g < 100:
            return "blue"
        if r > 150 and g > 150 and b < 120:
            return "yellow"
        return "unknown" 
      
    def detect_blocks(self):
        """
        Detect blocks and publish their positions and colors.
        """
        # If already completed, skip
        if self.completed:
            return
        # get the transformed pointcloud clusters
        clusters = self.get_cluster_positions()     
        self.get_logger().info(f"Clusters received for processing: {clusters}")
        if len(clusters) == 0:
            self.get_logger().info("No clusters to process.")
            return   
        self.get_logger().info(f"Number of clusters detected: {len(clusters)}")
        # Create a dictionary to hold detected block positions by color        
        detected_blocks = {
            'color': {},
            'pick_order': self._pick_order
        }
        
        seen_colors = set()# Store all detected colors in this scan
        
        for cluster in clusters:
            cluster_color = self.extract_color_names(cluster['color'])
            self.get_logger().info(f"Cluster color classified as: {cluster_color}")
            if( cluster_color == "unknown"):
                continue
            seen_colors.add(cluster_color)
            detected_blocks['color'][cluster_color] = cluster['position']

        msg = String()
        msg.data = json.dumps(detected_blocks)
        self.pub.publish(msg)
        self.get_logger().info(f'Published detected blocks: {msg}')
        # ----------------------------
        # CHECK COMPLETION CONDITION
        # ----------------------------
        self.found_colors.update(seen_colors)
        all_required = set(self._pick_order)

        if self.found_colors.issubset(all_required):
            self.get_logger().info("All blocks in pick_order detected. Stopping timer.")
            
            # Stop timer
            # self.timer.cancel()
            self.completed = True
            
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
