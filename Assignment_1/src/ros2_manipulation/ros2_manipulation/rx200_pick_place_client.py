#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion, Point
import math
from transforms3d.euler import euler2quat
from std_msgs.msg import Int32

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
        self.declare_parameter('object_height', value=0.2)
 
        self.send_gr_pose(self.get_parameter('start_state_gripper').value)
        self._pick_point = tuple(self.get_parameter('pick_point').value)
        self._place_point = tuple(self.get_parameter('place_point').value)
        self._object_height = self.get_parameter('object_height').value
        
        # subscription to the commanding node to get the state/position of the arm
        self.command_sub = self.create_subscription(Int32, '/pick_place_command', self.command_callback, 10)
        # State tracking
        self.current_state = 0
        self.get_logger().info('Node initialized successfully!')
    
    def send_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        distance = math.sqrt(x**2 + y**2)  # Horizontal distance from base
        # pitch is decided based on how close we are to the object and how low it is
        if distance < 0.2:
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

    def send_gr_pose(self, open=True):
        req = MotionPlanRequest()
        req.group_name = self.group_name_gripper
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.035 if open else 0.0
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

    # Callbacks 
    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt goal rejected')
            self.current_state = 0
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

    def command_callback(self, msg):
        if msg.data != self.current_state:  # Only process if new state is received
            self.current_state = msg.data
            self.get_logger().info(f'Received state {self.current_state}')
            if 1 <= self.current_state <= 8:
                self.run()
            elif self.current_state == 9:
                self.get_logger().info("PICK-AND-PLACE COMPLETE")

                
    # === Movement Sequence ===
    def run(self):
        x, y = self._pick_point
        z = self._object_height  # height, any object to be lifted from
        lift_height = 0.1 # fixing it to one step above the object position
        distance = math.sqrt(x**2 + y**2)  # Horizontal distance from base
        if distance > 0.45 or distance < 0.1 or z < 0.05:
            self.get_logger().error(f"Target ({x}, {y}, {z}) is unreachable: "
                                  f"Distance {distance:.2f}m exceeds max reach {0.45}m or "
                                  f"z {z}m is below min height {0.05}m")
            return None
        if self.current_state == 1:
        # 1. Before Pick: A step before the pick point 
            before_pick = Point(x=round(self._pick_point[0] - 0.1, 2), y=self._pick_point[1], z=z)
            self.get_logger().info(f"Moving before PICK point: {before_pick}")
            self.send_pose(before_pick.x, before_pick.y, before_pick.z)
        elif self.current_state == 2:                                                   
        # 2. Move to Pick point
            pick_point = Point(x=self._pick_point[0], y=self._pick_point[1], z=z)
            self.get_logger().info(f"MOVING TO PICK point: {pick_point}")
            self.send_pose(pick_point.x, pick_point.y, pick_point.z)
        elif self.current_state == 3:  
            # 3. Close gripper: hold the object
            self.get_logger().info("CLOSING GRIPPER")
            self.send_gr_pose(open=False)
        elif self.current_state == 4:  
            # 4. Lift
            above_pick = Point(x=self._pick_point[0], y=self._pick_point[1], z= round(z + lift_height,2))
            self.get_logger().info(f"LIFTING: {above_pick}")
            self.send_pose(above_pick.x, above_pick.y, above_pick.z)
        elif self.current_state == 5:  
            # 5. Go above Place point
            above_place = Point(x=self._place_point[0], y=self._place_point[1], z=round(z+ lift_height, 2))
            self.get_logger().info(f"Moving ABOVE PLACE point: {above_place}")
            self.send_pose(above_place.x, above_place.y, above_place.z)
        elif self.current_state == 6:  
            # 6. Descend to Place
            place_point = Point(x=self._place_point[0], y=self._place_point[1], z=z)
            self.get_logger().info(f"DESCENDING TO PLACE point: {place_point}")
            self.send_pose(place_point.x, place_point.y, place_point.z)
        elif self.current_state == 7:  
            # 7. Open gripper: release the object
            self.get_logger().info("OPENING GRIPPER")
            self.send_gr_pose(open=True)
        elif self.current_state == 8:  
            # 8. Lift: Back to above place            
            above_place = Point(x=self._place_point[0], y=self._place_point[1], z=round(z+ lift_height, 2))
            self.get_logger().info(f"LIFTING {above_place}")
            self.send_pose(above_place.x, above_place.y, above_place.z)  

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
