#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion, Point
import time
import math
from transforms3d.euler import euler2quat

class MoveItEEClient(Node):
    def __init__(self):
        super().__init__('rx200_pick_place')

        self._client = ActionClient(self, MoveGroup, '/move_action')
        while not self._client.wait_for_server(1.0):
            self.get_logger().warning('Waiting for Action Server...')

        self.group_name_arm = 'interbotix_arm'
        self.group_name_gripper = 'interbotix_gripper'
        self.ee_link = 'rx200/ee_gripper_link'
        self.base_link = 'rx200/base_link'
        self.gripper_joint = 'left_finger'

        # Declare parameters to be used from launch file
        self.declare_parameter('start_state_gripper', value=True)
        self.declare_parameter('pick_point', value=[0.1, 0.0])
        self.declare_parameter('place_point', value=[0.1, 0.12])
        self.declare_parameter('timeout_sec', value=30.0)
 
        self.send_gr_pose(self.get_parameter('start_state_gripper').value)
        self._pick_point = tuple(self.get_parameter('pick_point').value)
        self._place_point = tuple(self.get_parameter('place_point').value)
        self._timeout_sec = self.get_parameter('timeout_sec').value

        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.path_points = []
        
        self.get_logger().info('Node initialized successfully!')
    
    def send_pose(self, x, y, z, w = 0.0):
        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        distance = math.sqrt(x**2 + y**2)  # Horizontal distance from base
        # pitch is descided based on how close we are to the object and how low it is
        if distance < 0.3:
            pitch = math.pi / 2  # Pointing downwards
        else:
            pitch = 0.0  # Level orientation
        # yaw = tan-1(y/x) gives angle between x-axis and line joining origin to point (x,y) 
        # Stating how much the end-effector should rotate to face the point (x,y)
        # euler2quat converts euler angles to quaternion
        yaw = math.atan2(y, x) 
        q = euler2quat(0, pitch, yaw)
        pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])

        req = MotionPlanRequest()
        req.group_name = self.group_name_arm
        req.allowed_planning_time  = 10.0
        req.num_planning_attempts = 3 #try 3 times, always have to be int, no float

        pc = PositionConstraint()
        pc.header.frame_id = self.base_link
        pc.link_name = self.ee_link
        sp = SolidPrimitive() 
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]

        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [pose.pose]

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_link
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pc]
        goal_constraints.orientation_constraints = [oc]
        req.goal_constraints = [goal_constraints]


        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.look_around = False

        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

        return send_future

    def send_gr_pose(self, open=True):
        req = MotionPlanRequest()
        req.group_name = self.group_name_gripper
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.0 if open else 0.035
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = [jc]
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False


        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

        # Return the send future to allow optional synchronous waiting by the caller
        return send_future

    def send_pose_and_wait(self, x, y, z, w=0.0, timeout_sec: float = None):
        """Send a pose goal and block until the goal result is available (or timeout).
        """
        send_future = self.send_pose(x, y, z, w)
        if send_future is None:
            self.get_logger().error(f"Failed to send pose to ({x}, {y}, {z}) due to unreachable target")
            return None
        # Wait until send_future completes (gives a GoalHandle)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Failed to get goal handle (send_future returned None)')
            return None

        # Wait for the result future to complete
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        return result_future.result().result if result_future.done() else None  
    
    # Callbacks 
    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt goal rejected')
            return
        self.get_logger().info('MoveIt goal accepted')
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        state = getattr(feedback_msg.feedback, "state", "<unknown>")
        self.get_logger().info(f'[Feedback] {state}')

    def _result_cb(self, future):
        result = future.result().result
        code = getattr(result.error_code, 'val', 'unknown')
        self.get_logger().info(f'[Result] error_code {code}')

    # === Movement Sequence ===
    def run(self):
        # Check if the target is within reach
        x, y = self._pick_point
        z = 0.1  # Fixed height as threshold to avoid ground collision
        distance = math.sqrt(x**2 + y**2)  # Horizontal distance from base
        if distance > 0.45 or distance < 0.1 or z < 0.05:
            self.get_logger().error(f"Target ({x}, {y}, {z}) is unreachable: "
                                  f"Distance {distance:.2f}m exceeds max reach {0.45}m or "
                                  f"z {z}m is below min height {0.05}m")
            return None
        
        # 1. Before Pick: A step before the pick point 
        before_pick = Point(x=round(self._pick_point[0] - 0.1, 2), y=self._pick_point[1], z=z)
        self.get_logger().info(f"Moving before PICK point: {before_pick}")
        if not self.send_pose_and_wait(before_pick.x, before_pick.y, before_pick.z):
            return
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        
        # 2. Move to Pick point
        pick_point = Point(x=self._pick_point[0], y=self._pick_point[1], z=z)
        self.get_logger().info(f"MOVING TO PICK point: {pick_point}")
        if not self.send_pose_and_wait(pick_point.x, pick_point.y, pick_point.z):
            return
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))

        # 3. Close gripper: hold the object
        self.get_logger().info("CLOSING GRIPPER")
        self.send_gr_pose(open=True)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))

        # 4. Lift
        above_pick = Point(x=self._pick_point[0], y=round(self._pick_point[1] + 0.15, 2), z=z)
        self.get_logger().info(f"LIFTING: {above_pick}")
        if not self.send_pose_and_wait(above_pick.x, above_pick.y, above_pick.z):
            return
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))

        # 5. Above Place point, 0.15 indicate a random lifting height, since z is not used
        above_place = Point(x=self._place_point[0], y=round(self._place_point[1] + 0.15, 2), z=z)
        self.get_logger().info(f"Moving ABOVE PLACE point: {above_place}")
        if not self.send_pose_and_wait(above_place.x, above_place.y, above_place.z):
            return
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))
        
        # 6. Descend to Place
        place_point = Point(x=self._place_point[0], y=self._place_point[1], z=z)
        self.get_logger().info(f"DESCENDING TO PLACE point: {place_point}")
        if not self.send_pose_and_wait(place_point.x, place_point.y, place_point.z):
            return
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))
        
        # 7. Open gripper: release the object
        self.get_logger().info("OPENING GRIPPER")
        self.send_gr_pose(open=False)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))

        # 8. Lift: Back to above place
        self.get_logger().info(f"LIFTING {above_place}")
        if not self.send_pose_and_wait(above_place.x, above_place.y, above_place.z):
            return
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))       
        
        self.get_logger().info("PICK-AND-PLACE COMPLETE")
        
        self.get_logger().info(f"Moving to Home")
        if not self.send_pose_and_wait(0.0, 0.0, z):
            return
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))

def main():
    rclpy.init()
    node = MoveItEEClient()   
    try:
        # while rclpy.ok():
        node.run()            
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user, shutting down')
    finally:
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
