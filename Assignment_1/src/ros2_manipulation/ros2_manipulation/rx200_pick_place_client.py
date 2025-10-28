#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion
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
        self.ee_link = 'rx200/ee_gripper_link' #ee+ End Efector
        self.base_link = 'rx200/base_link'
        self.gripper_joint = 'left_finger'

        self.declare_parameter('start_state_gripper', value=True)
        # This is just for testing purpose, later we will change the points
        self.declare_parameter('point_a', value=[0.1, 0.0, 0.15])
        self.declare_parameter('point_b', value=[0.1, 0.12, 0.15])
        self.declare_parameter('point_c', value=[0.1, 0.24, 0.15])
        self.declare_parameter('point_d', value=[0.1, 0.36, 0.15])
        self.declare_parameter('relax_constraints', value=False)
        self.declare_parameter('timeout_sec', value=30.0)
 
        self.send_gr_pose(self.get_parameter('start_state_gripper').value)
        self._point_a = tuple(self.get_parameter('point_a').value)
        self._point_b = tuple(self.get_parameter('point_b').value)
        self._point_c = tuple(self.get_parameter('point_c').value)
        self._point_d = tuple(self.get_parameter('point_d').value)
        self._relax_constraints = self.get_parameter('relax_constraints').value
        self._timeout_sec = self.get_parameter('timeout_sec').value

        self.get_logger().info('Node initialized successfully!')

    def send_pose(self, x, y, z, w = 1.0):
        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        yaw = math.atan2(y, x)
        q = euler2quat(0, 0, yaw)
        pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        

        req = MotionPlanRequest()
        req.group_name = self.group_name_arm
        req.allowed_planning_time  = 5.0
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

    def send_pose_and_wait(self, x, y, z, w=1.0, timeout_sec: float = None):
        """Send a pose goal and block until the goal result is available (or timeout).

        Returns the MoveIt result object on success, or None on rejection/timeout.
        """
        send_future = self.send_pose(x, y, z, w)

        # Wait until send_future completes (gives a GoalHandle)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        goal_handle = send_future.result()
        if not goal_handle:
            self.get_logger().error('Failed to get goal handle (send_future returned None)')
            return None
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt goal rejected')
            return None

        # Wait for the result future to complete
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        result = None
        if result_future.done():
            # result_future.result() is a GoalResult, the actual moveit result is in .result
            try:
                result = result_future.result().result
            except Exception:
                # defensive: if shape differs, just return the raw future result
                result = result_future.result()

        return result

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


def main():
    rclpy.init()
    node = MoveItEEClient()
    points = [node._point_a, node._point_b, node._point_c, node._point_d]
    
    try:
        # This is just for testing purpose, later we will enable gripper control
        # while rclpy.ok():
        # Start with gripper open
        # node.send_gr_pose(open=True)
        # time.sleep(1.0)
        
        #TO DO: Will update the logic later, nedd to use compute_cartesian_path 
        # Move to point A and pick up object
        x, y, z = points[0]
        node.get_logger().info(f'Moving to pick point A: x={x}, y={y}, z={z}')
        result = node.send_pose_and_wait(x, y, z, timeout_sec=node._timeout_sec)
        if result is None:
            node.get_logger().warning('Move to point A failed or timed out')
            
        
        # Close gripper to grab object
        # node.send_gr_pose(open=False)
        time.sleep(2.0)
        
        # Move through points B and C
        for i, point in enumerate(points[1:3], 1):
            x, y, z = point
            node.get_logger().info(f'Moving to point {chr(65+i)}: x={x}, y={y}, z={z}')
            result = node.send_pose_and_wait(x, y, z, timeout_sec=node._timeout_sec)
            if result is None:
                node.get_logger().warning(f'Move to point {chr(65+i)} failed or timed out')
                break
            time.sleep(2.0)  # Short pause between movements
        
        # Move to point D and place object
        x, y, z = points[3]
        node.get_logger().info(f'Moving to place point D: x={x}, y={y}, z={z}')
        result = node.send_pose_and_wait(x, y, z, timeout_sec=node._timeout_sec)
        if result is None:
            node.get_logger().warning('Move to point D failed or timed out')
            
        
        # Open gripper to release object
        # node.send_gr_pose(open=True)
        # time.sleep(1.0)
        
        # Optional: Move arm slightly up after placing
        # node.send_pose_and_wait(x, y, z + 0.05, timeout_sec=node._timeout_sec)
        
        # Wait before starting next cycle
        time.sleep(2.0)
        node.get_logger().warning('*************END OF CYCLE*************')
            
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user, shutting down')
    finally:
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
