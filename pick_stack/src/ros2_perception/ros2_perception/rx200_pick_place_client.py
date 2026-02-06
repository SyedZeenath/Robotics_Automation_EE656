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
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState


import json

class MoveItEEClient(Node):
    def __init__(self):
        super().__init__('rx200_pick_place')

        # Publisher to signal readiness to perception node
        self.ready_pub = self.create_publisher(Bool, '/ready_for_perception', 1)
        
        self._client = ActionClient(self, MoveGroup, '/move_action')
        while not self._client.wait_for_server(2.0):
            self.get_logger().warning('Waiting for Action Server...')
        
        # ---------------
        # Robot-specific parameters
        # ---------------
        self.group_name_arm = 'interbotix_arm'
        self.group_name_gripper = 'interbotix_gripper'
        self.ee_link = 'rx200/ee_gripper_link'
        self.base_link = 'rx200/base_link'
        self.gripper_joint = 'left_finger'
        
        self.place_pos_red = Point(x=0.3, y=-0.2, z=0.03)    # stack position for red blocks
        self.place_pos_blue = Point(x=0.3, y=0.2, z=0.03)   # stack position for blue blocks
        self.place_pos_yellow = Point(x=0.2, y=0.1, z=0.03)  # stack position for yellow blocks
        self.block_height = 0.06       # height of each block, used to calculate z coordinate while stacking
        
        # ---------------
        # Subscriptions to joint states
        # ---------------
        self.current_joint_state = None
        self.create_subscription(JointState, '/rx200/joint_states', self.joint_state_cb, 10) # Subscribe to joint states
        
        self.max_retry = 0
        # ---------------
        # Go to home position
        # ---------------
        self.waiting_for_home = True
        self.home_pos()

        # ---------------
        # Subscribe to 'detected_blocks' from perception node
        # ---------------
        self.create_subscription(String, '/detected_blocks', self.blocks_callback, 10) 
        self.detected_blocks = {}

        self.get_logger().info('Pick & Place Control Node is Ready!')
    
    def publish_ready(self):
        msg = Bool()
        msg.data = True
        self.ready_pub.publish(msg)
        self.get_logger().info('Ready for perception.')

    def wait_move_group_server(self):
        if not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("MoveGroup server not ready, retrying...")
            return False
        return True
    
    def send_pose(self, x, y, z, pitch=0.65):
        if not self.wait_move_group_server():
            return

        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        distance = math.sqrt(x**2 + y**2)
        yaw = math.atan2(y, x) if distance >= 0.01 else 0.0 # Avoid division by zero for very close points
        roll = 0.0 # Roll is usually zero
        q = quaternion_from_euler(roll, pitch, yaw)   # Convert to quaternion

        # Construct pose object
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        req = MotionPlanRequest()
        req.group_name = self.group_name_arm
        req.allowed_planning_time  = 10.0
        req.num_planning_attempts = 3

        # Start from current real joint state (eliminates huge rotations)
        if self.current_joint_state:
            rs = RobotState()
            rs.joint_state = self.current_joint_state
            rs.is_diff = False
            req.start_state = rs
            
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
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.2  # Allow full yaw rotation
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
        
        self.last_pose_position = pose.pose.position
        self.last_pitch = pitch
        
        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)
    
    def send_gr_pose(self, open=True):
        req = MotionPlanRequest()
        req.group_name = self.group_name_gripper
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        req.start_state.is_diff = True
    
        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        
        jc.position = 0.055 if open else 0.030

        jc.tolerance_above = 0.002
        jc.tolerance_below = 0.002
        jc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = [jc]
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        # DO NOT replan gripper motions
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        # Slow execution to avoid current spike
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)
        # Return the send future to allow optional synchronous waiting by the caller
        return send_future

    def home_pos(self):
        self.get_logger().info('Moving to home position...')
        self.send_pose(0.2, 0.0, 0.35)   # high pose
    
    #---------------------    
    # Callbacks 
    #---------------------
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
        code = result.error_code.val
        self.get_logger().info(f'[Result] error_code {code}')
        if code == -4:
            self.get_logger().error(f"MoveGroup execution failed! error_code: {code}")
            # Retry same pose after short delay
            if not self.max_retry == 5:
                self.get_logger().info(f"Retrying last pose...Attempt:{self.max_retry}/5")
                self.send_pose(
                        self.last_pose_position.x,
                        self.last_pose_position.y,
                        self.last_pose_position.z,
                        self.last_pitch
                    )
                self.max_retry += 1
                
        else:
            if self.waiting_for_home:
                self.waiting_for_home = False
                self.get_logger().info("Home position reached.")
                self.publish_ready()
                return

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

    def joint_state_cb(self, msg):
        self.current_joint_state = msg
         
    # === Movement Sequence ===
    def run(self):
        self.get_logger().info("Starting Pick and Place Sequence")
        # check the color to pick
        for pick_color in self.pick_order:
            
            self.get_logger().info(f"*******Processing pick color: {pick_color}*******")
            
            if pick_color not in self.detected_blocks:
                self.get_logger().warning(f"{pick_color} block not detected, skipping to next color.")
                continue
            
            self._pick_point = self.detected_blocks[pick_color]
            self.get_logger().info(f"Detected {pick_color} block at {self._pick_point}")
            
            # ------------------
            # initialize pick & place points
            # ------------------
            x, y, z = self._pick_point
            if pick_color == 'red':
                self.place_pos = self.place_pos_red
            elif pick_color == 'blue':
                self.place_pos = self.place_pos_blue
            elif pick_color == 'yellow':
                self.place_pos = self.place_pos_yellow
            else:
                self.get_logger().warning(f"Unknown color {pick_color}, skipping.")
                continue
            xp, yp, zp = [self.place_pos.x, self.place_pos.y, self.place_pos.z]
            lift_height = 0.06 # fixing it to one step above the object position
            pitch = 1.57  # Pointing downwards 

            #----------------
            # Pick & Place Sequence
            #----------------

            # Open gripper: Before 
            self.get_logger().info("OPENING GRIPPER")
            self.send_gr_pose(open=True)
                
            time.sleep(6.0)
            # 1. Before Pick: A step before the pick point 
            self.get_logger().info(f"Moving before PICK point")
            self.send_pose(x, y, z + lift_height, pitch)
            time.sleep(5.0)  

            # 2. Move to Pick point
            self.get_logger().info(f"MOVING TO PICK point")
            self.send_pose(x, y, z, pitch)
            time.sleep(5.0)

            # 3. Close gripper: hold the object
            self.get_logger().info("CLOSING GRIPPER")
            self.send_gr_pose(open=False)
            time.sleep(6.0)
                    
            # 4. Lift
            self.get_logger().info(f"LIFTING")
            self.send_pose(x, y, z + lift_height, pitch)                
            time.sleep(5.0)

            # 5. Go above Place point
            self.get_logger().info(f"Moving ABOVE PLACE point")
            self.send_pose(xp, yp, zp + lift_height, pitch)                
            time.sleep(5.0)

            # 6. Descend to Place
            self.get_logger().info(f"DESCENDING TO PLACE point")
            self.send_pose(xp, yp, zp, pitch)                
            time.sleep(5.0)

            # 7. Open gripper: release the object
            self.get_logger().info("OPENING GRIPPER")
            self.send_gr_pose(open=True)                
            time.sleep(6.0)

            # 8. Lift: Back to above place            
            self.get_logger().info(f"LIFTING")
            self.send_pose(xp, yp, zp + lift_height, pitch)                
            time.sleep(5.0)
            
        self.get_logger().info('Pick & Place cycle complete!')
        self.home_pos()

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
