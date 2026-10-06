"""
mission_node.py — Student entry point.
"""

import os
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient

from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time

import yaml
import math
from pathlib import Path
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import Twist, TwistStamped
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from std_msgs.msg import Empty

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from action_msgs.msg import GoalStatus
import py_trees

# Selected implementation route: use a behavior tree from grade E onward.
# E/C: lecture-style "Make sure..." subtrees; A: complete reactive handling
# and AMCL readiness. ROS operations and observations remain student TODOs.
# E1: Nav2 action, action-result status, and behavior-tree imports are present.

# ─────────────────────────────────────────────────────────────────────────────
# WHERE THE MISSION GOES, PER GRADE
#
# Every grade collects the cube from the same source box. They differ only in
# which box it is placed on, and config/shelves.yaml holds all of them.
#
# The grade comes from a ROS parameter rather than being written in here, so it
# cannot silently disagree with the grade the simulation was launched with. Set
# both from one place:
#
#     GRADE=c pixi run mission            # world, odometry, localization
#     GRADE=c pixi run mission-node       # this node
#
# MissionNode logs the grade it is running as, so a mismatch is visible in the
# first line of output rather than showing up as the robot driving to the wrong
# box twenty metres away.
# ─────────────────────────────────────────────────────────────────────────────

SOURCE_BOX = 'shelf_7_ID11'

DROP_BOX_BY_GRADE = {
    'e': 'shelf_7_ID10',   # target-1, near the start
    'c': 'shelf_7_ID20',   # target-2, across the warehouse
    'a': 'shelf_7_ID20',   # target-2, same as C
}


def load_shelf(name: str) -> PoseStamped:
    for shelf in load_shelves():
        if shelf['name'] == name:
            return shelf['pose']
    known = [s['name'] for s in load_shelves()]
    raise KeyError(f"no box named '{name}' in shelves.yaml; known boxes: {known}")


def load_shelves(priority_first: bool = False) -> list[dict]:
    pkg_share = get_package_share_directory('warehouse_inventory_robot')
    yaml_path = Path(pkg_share) / 'config' / 'shelves.yaml'

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    shelves = []
    for shelf in data.get('shelves', []):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(shelf['pose']['x'])
        ps.pose.position.y = float(shelf['pose']['y'])
        ps.pose.position.z = 0.0
        yaw = float(shelf['pose'].get('yaw', 0.0))
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        shelves.append({
            'name':      shelf['name'],
            'marker_id': int(shelf['marker_id']),
            'priority':  shelf.get('priority', 'normal'),
            'pose':      ps,
        })

    if priority_first:
        shelves.sort(key=lambda s: 0 if s['priority'] == 'high' else 1)

    return shelves

def load_home_base() -> PoseStamped:
    pkg_share = get_package_share_directory('warehouse_inventory_robot')
    yaml_path = Path(pkg_share) / 'config' / 'shelves.yaml'

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    hb = data['home_base']
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.pose.position.x = float(hb['pose']['x'])
    ps.pose.position.y = float(hb['pose']['y'])
    yaw = float(hb['pose'].get('yaw', 0.0))
    ps.pose.orientation.z = math.sin(yaw / 2.0)
    ps.pose.orientation.w = math.cos(yaw / 2.0)
    return ps

class ConditionBehavior(py_trees.behaviour.Behaviour):
    """A lecture condition box: read state without sending a command."""

    def __init__(self, name, check):
        super().__init__(name=name)
        self.check = check

    def update(self):
        value = self.check()
        if not isinstance(value, bool):
            raise TypeError(f'{self.name}: condition must return bool')
        return (py_trees.common.Status.SUCCESS if value
                else py_trees.common.Status.FAILURE)


class ActionBehavior(py_trees.behaviour.Behaviour):
    """A lecture action box: delegate one non-blocking step to MissionNode."""

    def __init__(self, name, step):
        super().__init__(name=name)
        self.step = step

    def update(self):
        status = self.step()
        if status not in (py_trees.common.Status.RUNNING,
                          py_trees.common.Status.SUCCESS,
                          py_trees.common.Status.FAILURE):
            raise TypeError(f'{self.name}: action must return a BT status')
        return status

    # TODO E3-E6: Add initialise/terminate handling with each ROS operation.
    # On interruption, cancel and confirm the old goal has stopped before a
    # replacement action starts; also handle a late goal-acceptance response.
    # Do not erase a live future or assume stop(INVALID) cancels a ROS goal.


class UndockBehavior(ActionBehavior):
    def __init__(self, mission):
        super().__init__(name='Undock', step=mission.undock_robot)
        self.mission = mission

    def initialise(self):
        mission = self.mission
        mission._undock_send_future = None
        mission._undock_goal_handle = None
        mission._undock_result_future = None

class NavigateBehavior(ActionBehavior):
    def __init__(self, mission, pose):
        super().__init__(name='Navigate', step=lambda: mission.go_to_pose(pose))
        self.pose = pose
        self.mission = mission

    def initialise(self):
        mission = self.mission
        mission._nav_send_future = None
        mission._nav_goal_handle = None
        mission._nav_result_future = None

class MoveArmBehavior(ActionBehavior):
    def __init__(self, mission, angles, duration_sec=4):
        super().__init__(name='Move Arm', step=lambda: mission.move_arm_to_joint_angles(angles, duration_sec))
        self.angles = angles
        self.duration_sec = duration_sec
        self.mission = mission

    def initialise(self):
        mission = self.mission
        mission._arm_send_future = None
        mission._arm_goal_handle = None
        mission._arm_result_future = None

class VacuumBehavior(ActionBehavior):
    def __init__(self, mission, enable):
        super().__init__(name='Attach' if enable else 'Detach', step=lambda: mission.toggle_vacuum(enable))
        self.mission = mission

    def initialise(self):
        self.mission._vacuum_start_time = None

class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # Which grade this run is for. Read from the GRADE environment
        # variable, so one spelling works whether you go through
        # `GRADE=c pixi run mission-node` or call ros2 run yourself inside a
        # pixi shell. An explicit -p grade:=c overrides it. Use the same value
        # you launched the simulation with.
        self.declare_parameter('grade', os.environ.get('GRADE', 'e'))
        self.grade = str(self.get_parameter('grade').value).strip().lower()
        if self.grade not in DROP_BOX_BY_GRADE:
            self.get_logger().warn(
                f"Unknown grade '{self.grade}'; falling back to 'e'. "
                f"Valid grades: {sorted(DROP_BOX_BY_GRADE)}")
            self.grade = 'e'

        self.source_box = SOURCE_BOX
        self.drop_box = DROP_BOX_BY_GRADE[self.grade]

        self.get_logger().info(
            f"Mission node started for grade '{self.grade}'. "
            f'Collect from {self.source_box}, place on {self.drop_box}.')

        self._attach_pub = self.create_publisher(Empty, '/vacuum_gripper/attach', 10)
        self._detach_pub = self.create_publisher(Empty, '/vacuum_gripper/detach', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._undock_client = ActionClient(self, Undock, '/undock')
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._arm_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/lite6_traj_controller/follow_joint_trajectory'
        )

        self._clear_local_costmap_client = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self._clear_global_costmap_client = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')

        # Subscribe to /dock_status to detect when Create 3 is off the dock
        self._is_docked = True
        self._dock_status_sub = self.create_subscription(
            DockStatus, '/dock_status', self._dock_status_cb, qos_profile_sensor_data)

        # TODO E2: Add the NavigateToPose client and the state needed by BT leaves.
        #          Track pending requests, accepted goals, results, and timeouts.
        #          Keep current observations separate from task-stage completion.
        self._undocked = False
        self._undock_send_future = None
        self._undock_goal_handle = None
        self._undock_result_future = None
        self._undock_start_time = None
        self._undock_cancelling = False

        self._nav_goal_handle = None
        self._nav_result_future = None
        self._nav_send_future = None
        self._nav_retry_count = 0
        self._max_nav_retries = 3

        self._vacuum_start_time = None
        self._vacuum_enable = None
        self._vacuum_wait_secs = 1.5

        # Official Q&A 7 stow pose: [0, 0, 0, 0, -pi, 0] ensures balanced center of gravity
        # so wheels do not slip during undock rotation.
        self.safe_arm_angles = [0.0, 0.0, 0.0, 0.0, -math.pi, 0.0]
        self._arm_safe = False
        self._safe_move_started = False
        self._arm_send_future = None
        self._arm_goal_handle = None
        self._arm_result_future = None

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pos_tol = 0.1
        self.yaw_tol = math.radians(5)
        self.pose_max_age = 1.0

        self._holding_cube = False
        self._cube_delivered = False

        self.pick_arm_angles = [0.0, 0.87, 1.57, 0.0, -1.57, 0.0]
        self.lift_arm_angles = [0.0, 0.87, 1.57, 0.0, -1.57, 0.5]
        self.place_arm_angles = [0.0, 0.87, 1.57, 0.0, -1.57, 0.0]

    def _dock_status_cb(self, msg: DockStatus):
        self._is_docked = msg.is_docked

    def clear_all_costmaps(self):
        """Clear local and global costmaps to remove residual dock obstacles / ghosts."""
        self.get_logger().info("Clearing local and global costmaps...")
        req = ClearEntireCostmap.Request()
        if self._clear_local_costmap_client.service_is_ready():
            self._clear_local_costmap_client.call_async(req)
        if self._clear_global_costmap_client.service_is_ready():
            self._clear_global_costmap_client.call_async(req)

    def undock_robot(self):
        # E3: Non-blocking undock operation.
        # Following Assignment Q&A 7:
        # 1) Detach gripper at beginning to ensure no magnetic lock holds the robot
        # 2) Let Undock action run cleanly with stowed arm
        if self._undock_send_future is None:
            if not self._undock_client.server_is_ready():
                self.get_logger().warn("Undock action server not ready yet.")
                return py_trees.common.Status.RUNNING
            # Q&A 7: detach gripper before undocking
            self._detach_pub.publish(Empty())
            goal = Undock.Goal()
            self._undock_start_time = self.get_clock().now()
            self._undock_send_future = self._undock_client.send_goal_async(goal)
            return py_trees.common.Status.RUNNING

        if not self._undock_send_future.done():
            return py_trees.common.Status.RUNNING

        try:
            if self._undock_goal_handle is None:
                self._undock_goal_handle = self._undock_send_future.result()
            if not self._undock_goal_handle.accepted:
                self.get_logger().error("Undock goal was rejected!")
                return py_trees.common.Status.FAILURE

            if self._undock_result_future is None:
                self._undock_result_future = self._undock_goal_handle.get_result_async()

            if not self._undock_result_future.done():
                return py_trees.common.Status.RUNNING

            result = self._undock_result_future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED and not result.result.is_docked:
                self.get_logger().info("Undock completed successfully.")
                self.clear_all_costmaps()
                self._undocked = True
                return py_trees.common.Status.SUCCESS
            else:
                self.get_logger().error(f"Undock finished with status {result.status}, is_docked={result.result.is_docked}.")
                return py_trees.common.Status.FAILURE
        except Exception as e:
            self.get_logger().error(f"Exception during undock operation: {e}")
            return py_trees.common.Status.FAILURE

    def go_to_pose(self, pose_stamped):
        # E4: Implement NavigateToPose as a non-blocking BT operation.
        if self._nav_send_future is None:
            if not self._nav_client.server_is_ready():
                return py_trees.common.Status.RUNNING
            self.clear_all_costmaps()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info(
                f"Navigating to x: {pose_stamped.pose.position.x:.2f}, "
                f"y: {pose_stamped.pose.position.y:.2f} (retry: {self._nav_retry_count})"
            )
            goal = NavigateToPose.Goal()
            goal.pose = pose_stamped
            self._nav_send_future = self._nav_client.send_goal_async(goal)
            return py_trees.common.Status.RUNNING

        if not self._nav_send_future.done():
            return py_trees.common.Status.RUNNING

        try:
            if self._nav_goal_handle is None:
                self._nav_goal_handle = self._nav_send_future.result()
            if not self._nav_goal_handle.accepted:
                self.get_logger().error("Navigation goal was rejected!")
                return py_trees.common.Status.FAILURE
            if self._nav_result_future is None:
                self._nav_result_future = self._nav_goal_handle.get_result_async()
                return py_trees.common.Status.RUNNING
            if not self._nav_result_future.done():
                return py_trees.common.Status.RUNNING

            result = self._nav_result_future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Navigation succeeded.")
                self._nav_retry_count = 0
                return py_trees.common.Status.SUCCESS
            else:
                self.get_logger().warn(f"Navigation failed with status: {result.status}")
                if self._nav_retry_count < self._max_nav_retries:
                    self._nav_retry_count += 1
                    self.get_logger().info(
                        f"Retrying navigation ({self._nav_retry_count}/{self._max_nav_retries})..."
                    )
                    # Brief settle before retry to allow bt_navigator to clean up
                    settle_start = self.get_clock().now()
                    while (self.get_clock().now() - settle_start).nanoseconds < 1e9:
                        rclpy.spin_once(self, timeout_sec=0.1)
                    self.clear_all_costmaps()
                    self._nav_send_future = None
                    self._nav_goal_handle = None
                    self._nav_result_future = None
                    return py_trees.common.Status.RUNNING
                else:
                    self.get_logger().error(f"Navigation failed after {self._max_nav_retries} retries.")
                    self._nav_retry_count = 0
                    return py_trees.common.Status.FAILURE
        except Exception as e:
            self.get_logger().error(f"Exception during navigation operation: {e}")
            return py_trees.common.Status.FAILURE

    def toggle_vacuum(self, enable=True):
        # TODO E5: Adapt this helper for a responsive BT: publish once per phase
        #          and replace the blocking sleep with a non-blocking wait/check.
        #          A published Empty message is not proof of successful grasping.
        #          Track grasp/release phases so later ticks do not repeat them.
        if self._vacuum_start_time is None or self._vacuum_enable != enable:
            state = "ENGAGING" if enable else "RELEASING"
            self.get_logger().info(f'{state} vacuum gripper...')
            self._vacuum_start_time = self.get_clock().now()
            self._vacuum_enable = enable
            (self._attach_pub if enable else self._detach_pub).publish(Empty())
            return py_trees.common.Status.RUNNING

        elapsed = (self.get_clock().now() - self._vacuum_start_time).nanoseconds * 1e-9
        if elapsed < self._vacuum_wait_secs:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS


    def move_arm_to_joint_angles(self, angles, duration_sec=4):
        """Generic helper function to send the arm to any 6-DOF joint configuration."""
        # TODO E6: Reuse the trajectory construction, but separate sending from
        #          result polling before using this helper in a BT tick callback.
        #          Check final action status AND trajectory result error_code.
        #          Validate pick/place/safe poses; handle timeout and interruption.
        #          The synchronous implementation below is still the original
        #          template, not a completed non-blocking BT leaf.
        if self._arm_send_future is None:
            if not self._arm_client.server_is_ready():
                return py_trees.common.Status.RUNNING
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = [
                'arm_joint1', 'arm_joint2', 'arm_joint3', 
                'arm_joint4', 'arm_joint5', 'arm_joint6'
            ]
    
            point = JointTrajectoryPoint()
            point.positions = angles 
            point.time_from_start = Duration(sec=duration_sec, nanosec=0)
            goal_msg.trajectory.points.append(point)
    
            # Every wait below is bounded, so a misbehaving controller will show an
            # error instead of an indefinite hang with no output.
            self._arm_safe = False
            self._arm_send_future = self._arm_client.send_goal_async(goal_msg)
            return py_trees.common.Status.RUNNING

        if not self._arm_send_future.done():
            return py_trees.common.Status.RUNNING

        try:
            if self._arm_goal_handle is None:
                self._arm_goal_handle = self._arm_send_future.result()
            if not self._arm_goal_handle.accepted:
                self.get_logger().error("Arm trajectory goal was rejected!")
                return py_trees.common.Status.FAILURE
            if self._arm_result_future is None:
                self._arm_result_future = self._arm_goal_handle.get_result_async()
                return py_trees.common.Status.RUNNING
            if not self._arm_result_future.done():
                return py_trees.common.Status.RUNNING

            result = self._arm_result_future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code == 0:
                self.get_logger().info("Arm trajectory succeeded.")
                return py_trees.common.Status.SUCCESS
            else:
                self.get_logger().error(f"Arm trajectory failed with status: {result.status}, error_code: {result.result.error_code}")
                return py_trees.common.Status.FAILURE
        except Exception as e:
            self.get_logger().error(f"Exception during arm trajectory operation: {e}")
            return py_trees.common.Status.FAILURE




    # =========================================================================
    # LECTURE CONDITIONS AND ACTIONS -- STUDENT IMPLEMENTATION
    # =========================================================================

    def cube_at_destination(self):
        # TODO E2: Return bool from placement evidence, not the detach command.
        # For this mission, completion also requires the arm to be withdrawn.
        return self._cube_delivered

    def mark_cube_released(self):
        self._holding_cube = False
        return py_trees.common.Status.SUCCESS

    def mark_cube_delivered(self):
        self._cube_delivered = True
        self._arm_safe = True
        return py_trees.common.Status.SUCCESS

    def at_pose(self, pose):
        # TODO E2/E4: Compare current map-frame pose with position/yaw tolerance.
        # Unknown/stale pose is not success. Use AMCL/TF for A, not ground truth.
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time())
        except TransformException as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")
            return False
        stamp = Time.from_msg(transform.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9

        if age < 0.0 or age > self.pose_max_age:
            return False

        current_pos = transform.transform.translation
        current_orient = transform.transform.rotation
        target_pos = pose.pose.position
        target_orient = pose.pose.orientation

        pos_diff = math.hypot(
            current_pos.x - target_pos.x,
            current_pos.y - target_pos.y
        )

        current_yaw = math.atan2(
            2.0 * (current_orient.w * current_orient.z + current_orient.x * current_orient.y),
            1.0 - 2.0 * (current_orient.y ** 2 + current_orient.z ** 2)
        )
        target_yaw = math.atan2(
            2.0 * (target_orient.w * target_orient.z + target_orient.x * target_orient.y),
            1.0 - 2.0 * (target_orient.y ** 2 + target_orient.z ** 2)
        )

        yaw_diff = math.atan2(
            math.sin(current_yaw - target_yaw),
            math.cos(current_yaw - target_yaw)
        )

        return pos_diff < self.pos_tol and abs(yaw_diff) < self.yaw_tol

    def is_undocked(self):
        return self._undocked

    def arm_safe(self):
        # TODO E2/E6: Check current joint feedback against a validated safe pose.
        return self._arm_safe

    def localization_ready(self):
        # TODO A1: Check freshness/convergence and TF; do not return True blindly.
        raise NotImplementedError('A1: localization_ready observation')

    def pick_cube(self):
        # TODO E5/E6: Approach -> attach -> verify -> lift, without blocking.
        # Keep the base stopped and validate the source work pose while picking.
        return py_trees.composites.Sequence(
            name='Pick cube', memory=True, children=[
                MoveArmBehavior(self, self.pick_arm_angles),
                VacuumBehavior(self, enable=True),
                MoveArmBehavior(self, self.lift_arm_angles),
                ActionBehavior('Record holding cube', self.mark_cube_held)
            ]
        )

    def mark_cube_held(self):
        self._holding_cube = True
        return py_trees.common.Status.SUCCESS

    def holding_cube(self):
        return self._holding_cube

    def place_cube(self):
        # TODO E5/E6: Approach -> release -> verify -> withdraw, without blocking.
        # Check work pose/base stop throughout. Report SUCCESS only after the
        # cube is delivered AND the arm is withdrawn; do not repeat detachment.
        return py_trees.composites.Sequence(
            name='Place cube', memory=True, children=[
                MoveArmBehavior(self, self.place_arm_angles),
                VacuumBehavior(self, enable=False),
                ActionBehavior('Record cube released', self.mark_cube_released),
                MoveArmBehavior(self, self.safe_arm_angles),
                ActionBehavior('Record arm safe', self.mark_cube_delivered)
            ]
        )

    def move_arm_safe(self):
        # TODO E6: Move to validated safe joints while the base is stopped.
        # Adapt move_arm_to_joint_angles; do not call its blocking body in a tick.
        if not self._safe_move_started:
            self._safe_move_started = True
            self._arm_safe = False
            self._arm_send_future = None
            self._arm_goal_handle = None
            self._arm_result_future = None

        status = self.move_arm_to_joint_angles(self.safe_arm_angles)

        if status == py_trees.common.Status.SUCCESS:
            self._arm_safe = True

        if status != py_trees.common.Status.RUNNING:
            self._safe_move_started = False

        return status

    # =========================================================================
    # LECTURE TREE: ? = SELECTOR, -> = SEQUENCE
    # =========================================================================

    def make_sure_at(self, label, pose):
        """Expand 'At location?' using the lecture's PPA pattern."""
        undocked = py_trees.composites.Selector(
            name=f'Make sure undocked ({label})', memory=False, children=[
                ConditionBehavior('Undocked?', self.is_undocked),
                UndockBehavior(self),
            ])
        arm_safe = py_trees.composites.Selector(
            name=f'Make sure arm safe ({label})', memory=False, children=[
                ConditionBehavior('Arm safe?', self.arm_safe),
                ActionBehavior('Move arm safe', self.move_arm_safe),
            ])
        requirements = [undocked, arm_safe]
        if self.grade == 'a':
            requirements.append(ConditionBehavior(
                'Localization ready?', self.localization_ready))
        # Delivery has memory: explicitly recheck grasp during transportation.
        if label == 'destination':
            requirements.append(ConditionBehavior(
                'Holding cube while transporting?', self.holding_cube))
        requirements.append(NavigateBehavior(self, pose))
        move = py_trees.composites.Sequence(
            name=f'Move safely to {label}', memory=False, children=requirements)
        return py_trees.composites.Selector(
            name=f'Make sure at {label}', memory=False, children=[
                ConditionBehavior(f'At {label}?', lambda: self.at_pose(pose)),
                move,
            ])

    def build_tree(self, pick_pose, drop_pose):
        # E7: Lecture L8_BT_2026.pdf, PDF pages 20-21.
        # Construct fresh node objects for each parent; never share BT nodes.
        acquire = py_trees.composites.Sequence(
            name='Acquire cube', memory=False, children=[
                self.make_sure_at('source', pick_pose),
                self.pick_cube(),
            ])
        holding = py_trees.composites.Selector(
            name='Make sure holding cube', memory=False, children=[
                ConditionBehavior('Holding cube?', self.holding_cube),
                acquire,
            ])
        deliver = py_trees.composites.Sequence(
            name='Deliver cube', memory=True, children=[
                holding,
                self.make_sure_at('destination', drop_pose),
                self.place_cube(),
            ])
        # Deliberate difference from the lecture's fully reactive example:
        # remember delivery progress while Place releases and withdraws the arm.
        # Otherwise Holding becomes false during Place and pickup starts again.
        # Navigation guards above remain reactive; Place must guard its own pose.
        # TODO A1: Explain/refine this phase boundary for reactive recovery and
        # unknown-start localization. A tree shape alone does not complete A.
        root = py_trees.composites.Selector(
            name='Make sure cube delivered', memory=False, children=[
                ConditionBehavior('Cube delivered and arm withdrawn?',
                                  self.cube_at_destination),
                deliver,
            ])
        return py_trees.trees.BehaviourTree(root)

    def run_mission(self):
        pick_pose = load_shelf(self.source_box)
        drop_pose = load_shelf(self.drop_box)
        self.tree = self.build_tree(pick_pose, drop_pose)
        self.get_logger().info('\n' + py_trees.display.unicode_tree(self.tree.root))

        # E8: Readiness checks before ticking the mission tree.
        # Wait for action servers to avoid race condition on launch.
        self.get_logger().info("Waiting for action servers (/undock, /navigate_to_pose) to be ready...")
        while rclpy.ok() and not self._undock_client.wait_for_server(timeout_sec=1.0):
            rclpy.spin_once(self, timeout_sec=0.1)
        while rclpy.ok() and not self._nav_client.wait_for_server(timeout_sec=1.0):
            rclpy.spin_once(self, timeout_sec=0.1)

        # Wait for TF tree (map -> base_link) connection
        self.get_logger().info("Waiting for TF transform (map -> base_link) to become available...")
        tf_ready = False
        while rclpy.ok() and not tf_ready:
            try:
                self.tf_buffer.lookup_transform('map', 'base_link', Time())
                tf_ready = True
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info("All readiness checks passed. Starting mission tree ticks.")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.tree.tick()
            status = self.tree.root.status
            if status == py_trees.common.Status.SUCCESS:
                self.get_logger().info("Mission completed successfully.")
                break
            elif status == py_trees.common.Status.FAILURE:
                self.get_logger().error("Mission failed.")
                break

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()

    try:
        node.run_mission()
    except KeyboardInterrupt:
        node.get_logger().info("Mission interrupted by user.")
    except Exception as e:
        node.get_logger().fatal(f"Mission failed: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
