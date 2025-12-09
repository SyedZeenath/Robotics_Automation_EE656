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

class PickPlacePerception(Node):
    def __init__(self):
        super().__init__('rx200_pick_place_perception')
        self.base_link = 'rx200/base_link'
        self.ref_frame = 'camera_link'
        self.wrist_link = 'rx200/wrist_link'
        
        self.pub = self.create_publisher(String, '/detected_blocks', 10)
        
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

    def get_cluster_positions(self):
        """
        Get the estimated positions of all pointcloud clusters.
        """

        # clusters = self.srv_get_cluster_positions.call_async(self.clusterArray)
        # rclpy.spin_until_future_complete(self, clusters)
        clusters = [{'name': 'cluster_1', 'position': [0.00640977593138814, -0.0316816121339798, 0.6055483222007751], 'yaw': 0, 'color': [121.0, 107.0, 88.0], 'num_points': 666}, {'name': 'cluster_2', 'position': [0.0006334860809147358, -0.08940844982862473, 0.6497697830200195], 'yaw': 0, 'color': [203.0, 49.0, 56.0], 'num_points': 311}]
        if len(clusters) == 0:
            self.get_logger().warning('No clusters found...')
            return False, []
        num_clusters = len(clusters)

        # Get the cluster frame from the first cluster
        # cluster_frame = clusters[0].frame_id
        # Get the transform from the 'ref_frame' to the cluster frame (i.e. the camera's depth
        # frame) - known as T_rc
        self.get_logger().info(f"Looking up transform from '{self.wrist_link}' to '{self.ref_frame}'")
        # try:
        trans = self.tf_buffer.lookup_transform(
            target_frame=self.base_link,
            source_frame='camera_depth_optical_frame',
            time=rclpy.time.Time(),
            timeout=Duration(seconds=4.0)
        )
        self.get_logger().info(f"{trans} - Transform lookup successful!")
        # except (
        #     tf2_ros.LookupException,
        #     tf2_ros.ConnectivityException,
        #     tf2_ros.ExtrapolationException
        # ):
        #     self.get_logger().error(
        #         f"Failed to look up the transform from '{self.wrist_link}' to 'cluster_frame'."
        #     )
        #     return False, []
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

            cluster.position.x = p_ro[0]
            cluster.position.y = p_ro[1]
            cluster.position.z = p_ro[2]

        # publish transforms to the /tf tree for debugging purposes (only once)
        final_trans: List[TransformStamped] = []
        cluster_num = 1
        time_now = self.get_clock().now().to_msg()
        for cluster in clusters:
            trans = TransformStamped()
            trans.header.frame_id = self.base_link
            trans.header.stamp = time_now
            trans.child_frame_id = f'cluster_{str(cluster_num)}'
            trans.transform.translation.x = cluster.position.x
            trans.transform.translation.y = cluster.position.y
            trans.transform.translation.z = cluster.position.z
            trans.transform.rotation = Quaternion(x=0., y=0., z=0., w=1.)
            final_trans.append(trans)
            cluster_num += 1
        self.br.sendTransform(final_trans)

                # create a list of Python dictionaries to return to the user
        final_clusters = []
        for indx in range(num_clusters):
            name = final_trans[indx].child_frame_id
            x = final_trans[indx].transform.translation.x
            y = final_trans[indx].transform.translation.y
            z = final_trans[indx].transform.translation.z
            yaw = 0
            r = clusters[indx].color.r
            g = clusters[indx].color.g
            b = clusters[indx].color.b
            num_points = clusters[indx].num_points
            cluster = {
                'name': name,
                'position': [x, y, z],
                'yaw': yaw,
                'color': [r, g, b],
                'num_points': num_points
            }
            final_clusters.append(cluster)
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
        
        # clusters = [
        #     {
        #         'name': 'cluster_1',
        #         'position': [0.3, -0.2, 0.1],
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
        
        detected_blocks = {}
        for cluster in clusters:
            cluster_color = self.extract_color_names(cluster['color'])
            if( cluster_color == "unknown"):
                continue
            cluster_position = cluster['position']
            detected_blocks[cluster_color] = cluster_position
        
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


# [INFO] [1765212623.903559325] [rx200_pick_place_perception]: clusters detected 02: [{'name': 'cluster_1', 'position': [0.00640977593138814, -0.0316816121339798, 0.6055483222007751], 'yaw': 0, 'color': [121.0, 107.0, 88.0], 'num_points': 666}, {'name': 'cluster_2', 'position': [0.0006334860809147358, -0.08940844982862473, 0.6497697830200195], 'yaw': 0, 'color': [203.0, 49.0, 56.0], 'num_points': 311}]
# [INFO] [1765212623.903769306] [rx200_pick_place_perception]: Published detected blocks: {"unknown": [0.00640977593138814, -0.0316816121339798, 0.6055483222007751], "red": [0.0006334860809147358, -0.08940844982862473, 0.6497697830200195]}



# data: '{"red": [0.013220386579632759, 0.047836482524871826, 0.5752228498458862]}'