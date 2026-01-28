#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import TransformStamped, Quaternion
from interbotix_perception_msgs.srv import ClusterInfoArray
from tf_transformations import euler_from_quaternion
import tf2_ros
import json
import numpy as np
from interbotix_common_modules import angle_manipulation as ang

class PickPlacePerception(Node):
    def __init__(self):
        super().__init__('rx200_pick_place_perception')

        # Frames
        self.base_link = 'rx200/base_link'
        self.ref_frame = 'camera_link'
        self.wrist_link = 'rx200/wrist_link'

        # Publisher
        self.pub = self.create_publisher(String, '/detected_blocks', 10)
        self.arm_ready = False
        self.create_subscription(Bool, '/ready_for_perception', self.ready_callback, 10)
        
        #---------------
        # Parameters used for pick and place
        # 1. pick_order: order in which colors are to be picked
        #---------------
        self.declare_parameter('pick_order', value=['red', 'blue', 'yellow'])
        self._pick_order = self.get_parameter('pick_order').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.detect_blocks_requested = False

    def ready_callback(self, msg):
        if msg.data and not self.arm_ready:
            self.arm_ready = True
            self.get_logger().info("Arm ready signal received.")

            # TF broadcaster/listener
            self.br = tf2_ros.TransformBroadcaster(self)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

            # Service client
            self.srv_get_cluster_positions = self.create_client(ClusterInfoArray, '/pc_filter/get_cluster_positions')
            while not self.srv_get_cluster_positions.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for cluster service...")

            # Async state
            self.detect_blocks_requested = True

            self.process_requests()

            self.get_logger().info("PickPlacePerception Node Initialized")

    # -----------------------------
    # Parameter callback
    # -----------------------------
    def parameter_callback(self, params):
        if not self.arm_ready:
                self.get_logger().warn(
                    "Ignoring parameter change: arm not at home yet."
                )
                return SetParametersResult(successful=False)

        for param in params:
            if param.name == 'pick_order':
                self._pick_order = param.value
                self.detect_blocks_requested = True
                self.process_requests()

        return SetParametersResult(successful=True)

    # -----------------------------
    # Timer callback to process requests
    # -----------------------------
    def process_requests(self):
        if self.detect_blocks_requested:
            self.detect_blocks_requested = False
            self.detect_blocks_async()

    # -----------------------------
    # Async cluster request
    # -----------------------------
    def detect_blocks_async(self):
        self.get_logger().info("Requesting clusters from service...")
        req = ClusterInfoArray.Request()
        future = self.srv_get_cluster_positions.call_async(req)
        future.add_done_callback(self.on_clusters_received)

    # -----------------------------
    # Handle cluster response
    # -----------------------------
    def on_clusters_received(self, future):
        try:
            result = future.result()
            clusters = result.clusters
            if not clusters:
                self.get_logger().info("No clusters returned from service")
                return
            self.get_logger().info(f"Received {len(clusters)} clusters")
            self.process_clusters(clusters)
        except Exception as e:
            self.get_logger().error(f"Failed to get clusters: {e}")

    # -----------------------------
    # Cluster processing & TF
    # -----------------------------
    def process_clusters(self, clusters):
        cluster_frame = clusters[0].frame_id

        # Lookup transform base_link <- cluster_frame
        trans = self.safe_lookup_transform(self.base_link, cluster_frame)
        if trans is None:
            return

        x, y, z = trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z
        quat = trans.transform.rotation
        rpy = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        T_rc = ang.pose_to_transformation_matrix([x, y, z, rpy[0], rpy[1], rpy[2]])

        time_now = self.get_clock().now().to_msg()
        final_clusters = []

        for i, cluster in enumerate(clusters, start=1):
            pos_cam = np.array([cluster.position.x, cluster.position.y, cluster.position.z, 1])
            pos_base = T_rc @ pos_cam

            # Update cluster position
            cluster.position.x, cluster.position.y, cluster.position.z = pos_base[:3]

            # Publish TF
            t_msg = TransformStamped()
            t_msg.header.frame_id = self.base_link
            t_msg.header.stamp = time_now
            t_msg.child_frame_id = f'cluster_{i}'
            t_msg.transform.translation.x, t_msg.transform.translation.y, t_msg.transform.translation.z = pos_base[:3]
            t_msg.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            self.br.sendTransform(t_msg)

            # Save final cluster info
            final_clusters.append({
                'name': f'cluster_{i}',
                'position': pos_base[:3].tolist(),
                'yaw': 0,
                'color': [cluster.color.r, cluster.color.g, cluster.color.b],
                'num_points': cluster.num_points
            })

        self.publish_detected_blocks(final_clusters)

    # -----------------------------
    # Publish detected blocks
    # -----------------------------
    def publish_detected_blocks(self, clusters):
        detected_blocks = {'color': {}, 'pick_order': self._pick_order}
        for cluster in clusters:
            color_name = self.extract_color_names(cluster['color'])
            if color_name != "unknown":
                detected_blocks['color'][color_name] = cluster['position']

        msg = String()
        msg.data = json.dumps(detected_blocks)
        self.pub.publish(msg)
        self.get_logger().info(f"Published detected blocks: {msg.data}")

    # -----------------------------
    # Simple RGB color classifier
    # -----------------------------
    def extract_color_names(self, cluster_color):
        r, g, b = cluster_color
        if r > 150 and g < 120 and b < 120:
            return "red"
        if b > max(r, g) + 20 and b < 120 and r < 100 and g < 100:
            return "blue"
        if r > 150 and g > 150 and b < 120:
            return "yellow"
        return "unknown"

    # -----------------------------
    # Robust TF lookup
    # -----------------------------
    def safe_lookup_transform(self, target, source, attempts=10, delay=0.1):
        for i in range(attempts):
            try:
                return self.tf_buffer.lookup_transform(target, source, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed {source}->{target} attempt {i+1}/{attempts}: {e}")
                rclpy.spin_once(self, timeout_sec=delay)
        self.get_logger().error(f"TF lookup failed {source}->{target} after {attempts} attempts")
        return None

def main():
    rclpy.init()
    node = PickPlacePerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
