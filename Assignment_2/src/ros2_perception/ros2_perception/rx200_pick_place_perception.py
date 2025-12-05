#!/usr/bin/env python3
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

class PickPlacePerception(Node):
    def __init__(self):
        super().__init__('rx200_pick_place_perception')
        self.base_link = 'rx200/base_link'
        self.ref_frame = 'camera_link'
        
        self.pub = self.create_publisher(String, '/detected_blocks', 10)
        
        self.srv_get_cluster_positions = self.create_client(
            ClusterInfoArray,
            f'/pc_filter/get_cluster_positions'
        )
        while not self.srv_get_cluster_positions.wait_for_service(1.0):
            self.get_logger().info(
                f"Waiting for services '{self.srv_get_cluster_positions.srv_name}', come up.")
        self.clusterArray = ClusterInfoArray.Request()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(f"Perception Node Initialised")    
        self.detect_blocks()    

    def get_cluster_positions(self):
        """
        Get the estimated positions of all pointcloud clusters.
        """

        clusters = self.srv_get_cluster_positions.call_async(self.clusterArray)
        rclpy.spin_until_future_complete(self, clusters)
        clusters = clusters.result().clusters
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
            self.get_logger().error(
                f"Failed to look up the transform from '{self.ref_frame}' to '{cluster_frame}'."
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



#THIS IS JUST FOR UNDERSTANDING PURPOSES
#  response:
# interbotix_perception_msgs.srv.ClusterInfoArray_Response(clusters=[interbotix_perception_msgs.msg.ClusterInfo(frame_id='camera_depth_optical_frame', position=geometry_msgs.msg.Point(x=-0.20518581569194794, y=0.0009221067884936929, z=0.41817721724510193), yaw=0.0, color=std_msgs.msg.ColorRGBA(r=222.0, g=182.0, b=108.0, a=0.0), min_z_point=geometry_msgs.msg.Point(x=-0.19437815248966217, y=0.004924398381263018, z=0.4018386900424957), num_points=100), interbotix_perception_msgs.msg.ClusterInfo(frame_id='camera_depth_optical_frame', position=geometry_msgs.msg.Point(x=-0.25981006026268005, y=0.15942522883415222, z=0.4832031726837158), yaw=0.0, color=std_msgs.msg.ColorRGBA(r=139.0, g=144.0, b=139.0, a=0.0), min_z_point=geometry_msgs.msg.Point(x=-0.2149903029203415, y=0.15500614047050476, z=0.4449998736381531), num_points=79), interbotix_perception_msgs.msg.ClusterInfo(frame_id='camera_depth_optical_frame', position=geometry_msgs.msg.Point(x=-0.26035547256469727, y=0.02977876178920269, z=0.5097497701644897), yaw=0.0, color=std_msgs.msg.ColorRGBA(r=222.0, g=182.0, b=112.0, a=0.0), min_z_point=geometry_msgs.msg.Point(x=-0.2772863805294037, y=0.0325452946126461, z=0.4949016571044922), num_points=79), interbotix_perception_msgs.msg.ClusterInfo(frame_id='camera_depth_optical_frame', position=geometry_msgs.msg.Point(x=-0.1557665765285492, y=0.04978485777974129, z=0.5068286657333374), yaw=0.0, color=std_msgs.msg.ColorRGBA(r=179.0, g=141.0, b=75.0, a=0.0), min_z_point=geometry_msgs.msg.Point(x=-0.13188007473945618, y=0.04913558438420296, z=0.49133336544036865), num_points=66)])



#Error I was getting:

# [INFO] [1764957205.118426402] [rx200_pick_place_perception]: Number of clusters detected: 2
# Traceback (most recent call last):
#   File "/home/master26/Robotics_Automation_EE656/Assignment_2/install/ros2_perception/lib/ros2_perception/pick_place_perception", line 33, in <module>
#     sys.exit(load_entry_point('ros2-perception==0.0.0', 'console_scripts', 'pick_place_perception')())
#   File "/home/master26/Robotics_Automation_EE656/Assignment_2/install/ros2_perception/lib/python3.10/site-packages/ros2_perception/rx200_pick_place_perception.py", line 140, in main
#     node = PickPlacePerception()   
#   File "/home/master26/Robotics_Automation_EE656/Assignment_2/install/ros2_perception/lib/python3.10/site-packages/ros2_perception/rx200_pick_place_perception.py", line 37, in __init__
#     self.detect_blocks()    
#   File "/home/master26/Robotics_Automation_EE656/Assignment_2/install/ros2_perception/lib/python3.10/site-packages/ros2_perception/rx200_pick_place_perception.py", line 128, in detect_blocks
#     cluster_color = self.extract_color_names(cluster['color'])
# TypeError: 'bool' object is not subscriptable
# [ros2run]: Process exited with failure 1