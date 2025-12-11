#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
import time
import math
from tf_transformations import quaternion_from_euler
from std_msgs.msg import String


import json

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
        
        # stack positions
        self.stack_pos = Point(x=0.3, y=0.2, z=0.1)
        self.block_height = 0.05 # height of each block, used to calculate z coordinate while stacking
                
        # Declare parameters to be used from launch file
        self.declare_parameter('start_state_gripper', value=True)
 
        self.send_gr_pose(self.get_parameter('start_state_gripper').value)
        
        self.perception_sub = self.create_subscription(String, '/detected_blocks', self.blocks_callback, 10)
        self.detected_blocks = {}
        self.get_logger().info('Node initialized successfully!')
        top_point = Point(x=0.1, y=0.0, z=0.4)
        self.send_pose(top_point.x, top_point.y, top_point.z)

    
    def send_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        distance = math.sqrt(x**2 + y**2)

        # Avoid division by zero for very close points
        if distance < 0.01:
            yaw = 0.0
        else:
            yaw = 0.0

        # Adaptive pitch: point naturally toward the target based on vertical offset
        pitch = math.pi / 6  #math.atan2(z, distance)

        # Limit pitch to reasonable range to prevent IK over-extension
        # max_pitch = 1.0  # ~57 degrees
        # min_pitch = -0.5 # ~-28 degrees
        # pitch = max(min(pitch, max_pitch), min_pitch)

        # Roll is usually zero
        roll = 0.0
        self.get_logger().info(f"Computed Euler angles - Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
        # Convert to quaternion
        q = quaternion_from_euler(roll, pitch, yaw)

        # Construct pose object
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        req = MotionPlanRequest()
        req.group_name = self.group_name_arm
        req.allowed_planning_time  = 10.0
        req.num_planning_attempts = 3 #try 3 times, always have to be int, no float

        pc = PositionConstraint()
        pc.header.frame_id = self.base_link
        pc.link_name = self.ee_link
        sp = SolidPrimitive() 
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.03]

        # pose_position_only = Pose()
        # pose_position_only.position.x = x
        # pose_position_only.position.y = y
        # pose_position_only.position.z = z
        # pose_position_only.orientation.w = 1.0   # identity, ignored
        # pc.constraint_region.primitive_poses = [pose_position_only]

        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [pose.pose]

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_link
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.35
        oc.absolute_y_axis_tolerance = 0.35
        oc.absolute_z_axis_tolerance = 0.9
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

    def send_gr_pose(self, open=True):
        req = MotionPlanRequest()
        req.group_name = self.group_name_gripper
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.05 if open else 0.02
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

    def blocks_callback(self, msg):
        """
        Callback function to process detected blocks from perception node.
        The message is expected to contain block color and position information.
        """
        self.callback_res = json.loads(msg.data)
        self.get_logger().info(f"Received detected blocks data: {self.callback_res}")
        self.detected_blocks = self.callback_res['color']
        self.pick_order = self.callback_res['pick_order']    
        self.run()


            
    # === Movement Sequence ===
    def run(self):

        self.get_logger().info("Starting Pick and Place Sequence")
        # check the color to pick
        self.get_logger().info(f"Detected Blocks: {self.detected_blocks}")
        for pick_color in self.pick_order:
            pick_color = pick_color.replace("\\","").replace("'", "")
            self.get_logger().info(f"*******Processing pick color: {pick_color}*******")
            if pick_color not in self.detected_blocks:
                self.get_logger().warning(f"{pick_color} block not detected, skipping to next color.")
                continue
            self._pick_point = self.detected_blocks[pick_color]
            self.get_logger().info(f"Detected {pick_color} block at {self._pick_point}")
            
            x, y, z = self._pick_point
            xp, yp, zp = [self.stack_pos.x, self.stack_pos.y, self.stack_pos.z]
            z_stack = zp + self.block_height * self.pick_order.index(pick_color) # calculating height for stacking block one over the other
            lift_height = 0.1 # fixing it to one step above the object position

            distance = math.sqrt(x**2 + y**2)  # Horizontal distance from base
            # if distance > 0.45 or distance < 0.1 or z < 0.05:
            #     self.get_logger().error(f"Target ({x}, {y}, {z}) is unreachable: "
            #                         f"Distance {distance:.2f}m out of reach or "
            #                         f"z {z}m is below min height {0.05}m")
            #     return None

        
            # 1. Before Pick: A step before the pick point 
            before_pick = Point(x=round(x - 0.1, 2), y=y, z=z)
            self.get_logger().info(f"Moving before PICK point: {before_pick}")
            self.send_pose(before_pick.x, before_pick.y, before_pick.z)
            time.sleep(5.0)                                
            # 2. Move to Pick point
            pick_point = Point(x=x, y=y, z=z-0.02)
            self.get_logger().info(f"MOVING TO PICK point: {pick_point}")
            self.send_pose(pick_point.x, pick_point.y, pick_point.z)
            time.sleep(5.0)
            # 3. Close gripper: hold the object
            self.get_logger().info("CLOSING GRIPPER")
            self.send_gr_pose(open=False)
            time.sleep(7.0)
            # 4. Lift
            above_pick = Point(x=x, y=y, z= round(z + lift_height,2))
            self.get_logger().info(f"LIFTING: {above_pick}")
            self.send_pose(above_pick.x, above_pick.y, above_pick.z)
            time.sleep(5.0)
            # 5. Go above Place point
            # above_place = Point(x=xp, y=yp, z=round(z_stack + lift_height, 2))
            # self.get_logger().info(f"Moving ABOVE PLACE point: {above_place}")
            # self.send_pose(above_place.x, above_place.y, above_place.z)
            # time.sleep(5.0)
            # 6. Descend to Place
            place_point = Point(x=xp, y=yp, z=z_stack)
            self.get_logger().info(f"DESCENDING TO PLACE point: {place_point}")
            self.send_pose(place_point.x, place_point.y, place_point.z)
            time.sleep(5.0)
            # 7. Open gripper: release the object
            self.get_logger().info("OPENING GRIPPER")
            self.send_gr_pose(open=True)
            time.sleep(7.0)
            # 8. Lift: Back to above place            
            above_place = Point(x=xp, y=yp, z=round(z_stack + lift_height, 2))
            self.get_logger().info(f"LIFTING {above_place}")
            self.send_pose(above_place.x, above_place.y, above_place.z)  
            
            time.sleep(5.0)

def main():
    rclpy.init()
    node = MoveItEEClient()   
    try:
        rclpy.spin(node)          
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
 
 
if __name__ == '__main__':
    main()
