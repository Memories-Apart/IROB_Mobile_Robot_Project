"""
mission_node.py — Student entry point.
"""

import os
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient

import yaml
import math
from pathlib import Path
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import Twist, TwistStamped
from ament_index_python.packages import get_package_share_directory
from irobot_create_msgs.action import Undock
from std_msgs.msg import Empty

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# TODO: Add any other necessary imports (e.g., for Nav2 actions, or behavior tree libraries).
import py_trees
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import JointState, LaserScan
from irobot_create_msgs.msg import DockStatus
from std_srvs.srv import Empty as EmptyService

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
        self._undock_client = ActionClient(self, Undock, '/undock')
        self._arm_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/lite6_traj_controller/follow_joint_trajectory'
        )

        # TODO: Define other necessary subscribers, publishers, and action clients (e.g., for navigation with Nav2).
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._localization_client = self.create_client(
            EmptyService, '/reinitialize_global_localization')
        self._nav_parameters = AsyncParameterClient(self, '/controller_server')
        self._nav_config_future = None
        self._cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self._joint_state = None
        self._dock_status = None
        self._scan = None
        self._amcl_pose = None
        self._joint_sub = self.create_subscription(
            JointState, '/joint_states',
            lambda msg: setattr(self, '_joint_state', msg) if 'arm_joint1' in msg.name else None,
            qos_profile_sensor_data)
        self._dock_sub = self.create_subscription(
            DockStatus, '/dock_status',
            lambda msg: setattr(self, '_dock_status', msg), qos_profile_sensor_data)
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan',
            lambda msg: setattr(self, '_scan', msg), qos_profile_sensor_data)
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            lambda msg: setattr(self, '_amcl_pose', msg), 10)

        self._undocked = False
        self._undock_send_future = None
        self._undock_goal_handle = None
        self._undock_result_future = None

        self._nav_goal_handle = None
        self._nav_result_future = None
        self._nav_send_future = None
        self._nav_cancel_requested = False
        self._nav_cancel_future = None
        self._move_requested = None

        self._arm_send_future = None
        self._arm_goal_handle = None
        self._arm_result_future = None
        self._localized = self.grade != 'a'
        self._holding_cube = False
        self._cube_delivered = False

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pos_tol = 0.04
        self.yaw_tol = math.radians(4)
        self.pose_max_age = 1.0
        self._vacuum_wait_secs = 1.5

        # FK-based poses for the supplied source waypoint and target-2 box.
        # Check the trajectories and cube placement in Gazebo before examination.
        self.safe_arm_angles = [0.0, -0.77383, 0.60764, 0.0, 1.38147, 0.0]
        self.pick_arm_angles = [-0.058756, 1.316914, 1.598426, 0.0, 0.281512, 0.0]
        self.lift_arm_angles = [-0.058756, 0.691309, 1.389453, 0.0, 0.698143, 0.0]
        self.place_arm_angles = list(self.pick_arm_angles)


    def undock_robot(self):
        # TODO: Implement undocking logic using the Undock action client (self._undock_client).
        #       Return True once the base is undocked, False if it refused.
        if self._undock_send_future is None:
            if self._dock_status is not None and not self._dock_status.is_docked:
                self._undocked = True
                return py_trees.common.Status.SUCCESS
            if not self._undock_client.server_is_ready():
                return py_trees.common.Status.RUNNING
            self._undock_started = time.monotonic()
            self._undock_send_future = self._undock_client.send_goal_async(Undock.Goal())

        if time.monotonic() - self._undock_started > 120.0:
            self.get_logger().error('Undock timed out.')
            return py_trees.common.Status.FAILURE
        if not self._undock_send_future.done():
            return py_trees.common.Status.RUNNING

        if self._undock_goal_handle is None:
            self._undock_goal_handle = self._undock_send_future.result()
            if not self._undock_goal_handle.accepted:
                self.get_logger().error('Undock goal was rejected.')
                return py_trees.common.Status.FAILURE
            self._undock_result_future = self._undock_goal_handle.get_result_async()

        if not self._undock_result_future.done():
            return py_trees.common.Status.RUNNING

        result = self._undock_result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.is_docked:
            self.get_logger().error('Undock failed.')
            return py_trees.common.Status.FAILURE

        self._undocked = True
        self._undock_send_future = None
        self._undock_goal_handle = None
        self._undock_result_future = None
        return py_trees.common.Status.SUCCESS


    def go_to_pose(self, pose_stamped):
        #pose_stamped.header.stamp = self.get_clock().now().to_msg()
        #self.get_logger().info(f"Navigating to x: {pose_stamped.pose.position.x}, y: {pose_stamped.pose.position.y}")
        # TODO: Implement navigation to the given pose using Nav2's NavigateToPose action.
        #       Return True once the robot has arrived, False if it did not. Callers
        #       read the return value as "did this work", so falling off the end and
        #       returning None counts as failure.
        # Tighten the supplied Nav2 goal tolerances for the fixed arm poses.
        if self._nav_config_future is None:
            if not self._nav_parameters.services_are_ready():
                return py_trees.common.Status.RUNNING
            self._nav_config_future = self._nav_parameters.set_parameters([
                Parameter('general_goal_checker.xy_goal_tolerance', value=0.025),
                Parameter('general_goal_checker.yaw_goal_tolerance', value=0.035),
            ])
            return py_trees.common.Status.RUNNING
        if not self._nav_config_future.done():
            return py_trees.common.Status.RUNNING
        if not all(r.successful for r in self._nav_config_future.result().results):
            self.get_logger().error('Nav2 rejected the goal tolerances.')
            return py_trees.common.Status.FAILURE

        if self._nav_send_future is None:
            if not self._nav_client.server_is_ready():
                return py_trees.common.Status.RUNNING
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            goal = NavigateToPose.Goal()
            goal.pose = pose_stamped
            self._nav_started = time.monotonic()
            self._nav_send_future = self._nav_client.send_goal_async(goal)
            self.get_logger().info(
                f'Navigating to ({pose_stamped.pose.position.x}, '
                f'{pose_stamped.pose.position.y}).')
            return py_trees.common.Status.RUNNING

        if time.monotonic() - self._nav_started > 600.0:
            self.get_logger().error('Navigation timed out.')
            return py_trees.common.Status.FAILURE
        if not self._nav_send_future.done():
            return py_trees.common.Status.RUNNING

        if self._nav_goal_handle is None:
            self._nav_goal_handle = self._nav_send_future.result()
            if not self._nav_goal_handle.accepted:
                self.get_logger().error('Navigation goal was rejected.')
                return py_trees.common.Status.FAILURE
            self._nav_result_future = self._nav_goal_handle.get_result_async()

        if not self._nav_result_future.done():
            return py_trees.common.Status.RUNNING

        result = self._nav_result_future.result()
        self._nav_send_future = None
        self._nav_goal_handle = None
        self._nav_result_future = None
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        self.get_logger().error(f'Navigation failed with status {result.status}.')
        return py_trees.common.Status.FAILURE

    def toggle_vacuum(self, enable=True):
        state = "ENGAGING" if enable else "RELEASING"
        self.get_logger().info(f'{state} vacuum gripper...')
        (self._attach_pub if enable else self._detach_pub).publish(Empty())
        time.sleep(1.5)

    def move_arm_to_joint_angles(self, angles, duration_sec=4):
        """Generic helper function to send the arm to any 6-DOF joint configuration."""
        if not self._arm_client.wait_for_server(timeout_sec=120.0):
            self.get_logger().error(
                'Arm action server /lite6_traj_controller/follow_joint_trajectory '
                'never appeared. Is lite6_traj_controller active? Check with: '
                'ros2 control list_controllers')
            return False


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
        send_goal_future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=15.0)
        if not send_goal_future.done():
            self.get_logger().error('Arm trajectory goal was never acknowledged!')
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm trajectory goal was rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=180.0)
        if not result_future.done():
            self.get_logger().error('Arm trajectory did not finish in time!')
            return False
        return True


    # =========================================================================
    # THE MAIN MISSION SEQUENCE
    # =========================================================================

    def run_mission(self):
        self.get_logger().info('Starting mission.')

        # Ensure gripper starts in a known-detached state
        time.sleep(1.0)  # allow ROS->bridge->gz discovery to complete across all hops
        self._detach_pub.publish(Empty())
        time.sleep(1.0)

        # Load waypoints. Which box the cube is placed on depends on the grade,
        # so both boxes are looked up BY NAME through the constants at the top
        # of this file rather than by their position in shelves.yaml. Indexing
        # into the list instead would tie the mission to the order of that file
        # and quietly send the robot to the wrong box when it changed.
        home_base = load_home_base()
        pick_pose = load_shelf(self.source_box)
        drop_pose = load_shelf(self.drop_box)

        # TODO: Implement the mission logic (either State Machine or Behavior Tree).
        mission = self
        status = py_trees.common.Status

        def seconds():
            return mission.get_clock().now().nanoseconds / 1e9

        def fresh(message):
            if message is None:
                return False
            stamp = Time.from_msg(message.header.stamp).nanoseconds / 1e9
            return 0.0 <= seconds() - stamp <= mission.pose_max_age

        def yaw(q):
            return math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        def robot_pose(frame='map'):
            try:
                transform = mission.tf_buffer.lookup_transform(frame, 'base_link', Time())
                return transform if fresh(transform) else None
            except TransformException:
                return None

        def at_pose(pose):
            if not mission._localized or mission._move_requested is not None:
                return False
            transform = robot_pose()
            if transform is None:
                return False
            position = transform.transform.translation
            angle = yaw(transform.transform.rotation) - yaw(pose.pose.orientation)
            angle = math.atan2(math.sin(angle), math.cos(angle))
            return (math.hypot(position.x - pose.pose.position.x,
                               position.y - pose.pose.position.y) <= mission.pos_tol
                    and abs(angle) <= mission.yaw_tol)

        def command_turn(speed=0.0):
            command = TwistStamped()
            command.header.stamp = mission.get_clock().now().to_msg()
            command.header.frame_id = 'base_link'
            command.twist.angular.z = speed
            mission._cmd_pub.publish(command)

        def arm_is_safe():
            if mission._arm_send_future is not None or not fresh(mission._joint_state):
                return False
            joints = dict(zip(mission._joint_state.name, mission._joint_state.position))
            return all(abs(joints.get(f'arm_joint{i + 1}', math.inf) - angle) < 0.04
                       for i, angle in enumerate(mission.safe_arm_angles))

        def cancel_navigation():
            # Keep the requested destination while preparation interrupts the tree.
            mission._nav_cancel_requested = True
            if mission._nav_send_future is None:
                mission._nav_cancel_requested = False
                mission._nav_cancel_future = None
                return True
            if not mission._nav_send_future.done():
                return False
            if mission._nav_goal_handle is None:
                mission._nav_goal_handle = mission._nav_send_future.result()
                if mission._nav_goal_handle.accepted:
                    mission._nav_result_future = mission._nav_goal_handle.get_result_async()
            if mission._nav_goal_handle.accepted:
                if mission._nav_cancel_future is None:
                    mission._nav_cancel_future = mission._nav_goal_handle.cancel_goal_async()
                if not mission._nav_result_future.done():
                    return False
            mission._nav_send_future = None
            mission._nav_goal_handle = None
            mission._nav_result_future = None
            mission._nav_cancel_future = None
            mission._nav_cancel_requested = False
            return True

        def move_arm(angles, via=None):
            if mission._arm_send_future is None:
                if not mission._arm_client.server_is_ready():
                    return status.RUNNING
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = [f'arm_joint{i}' for i in range(1, 7)]
                targets = [angles] if via is None else [via, angles]
                for i, target in enumerate(targets):
                    point = JointTrajectoryPoint()
                    point.positions = list(target)
                    point.time_from_start = Duration(sec=4 * (i + 1), nanosec=0)
                    goal.trajectory.points.append(point)
                mission._arm_started = time.monotonic()
                mission._arm_send_future = mission._arm_client.send_goal_async(goal)
                return status.RUNNING
            if time.monotonic() - mission._arm_started > 120.0:
                mission.get_logger().error('Arm trajectory timed out.')
                return status.FAILURE
            if not mission._arm_send_future.done():
                return status.RUNNING
            if mission._arm_goal_handle is None:
                mission._arm_goal_handle = mission._arm_send_future.result()
                if not mission._arm_goal_handle.accepted:
                    mission.get_logger().error('Arm trajectory was rejected.')
                    return status.FAILURE
                mission._arm_result_future = mission._arm_goal_handle.get_result_async()
            if not mission._arm_result_future.done():
                return status.RUNNING
            result = mission._arm_result_future.result()
            mission._arm_send_future = None
            mission._arm_goal_handle = None
            mission._arm_result_future = None
            if (result.status != GoalStatus.STATUS_SUCCEEDED
                    or result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                mission.get_logger().error('Arm trajectory failed.')
                return status.FAILURE
            return status.SUCCESS

        def move_to(pose):
            if mission._move_requested is None:
                mission._move_requested = pose
                return status.RUNNING
            if mission._nav_cancel_requested and not cancel_navigation():
                return status.RUNNING
            if prepareMove.status != status.SUCCESS:
                return status.RUNNING
            result = mission.go_to_pose(pose)
            if result == status.SUCCESS:
                mission._move_requested = None
                if not at_pose(pose):
                    mission.get_logger().error('Navigation ended outside the manipulation tolerance.')
                    return status.FAILURE
                command_turn()
            return result

        class CheckSafe(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)

            def update(self):
                if mission._move_requested is None or not mission._undocked:
                    return status.SUCCESS
                if not fresh(mission._scan):
                    return status.FAILURE
                ranges = [r for r in mission._scan.ranges
                          if not math.isnan(r) and r >= mission._scan.range_min]
                return status.SUCCESS if ranges and min(ranges) > 0.24 else status.FAILURE

        checkSafe = CheckSafe("CheckSafe")

        class AovidCollision(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)

            def update(self):
                cancel_navigation()
                command_turn()
                return status.RUNNING

        aovidCollision = AovidCollision("AovidCollision")

        class CheckNoMove(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)

            def update(self):
                return status.SUCCESS if mission._move_requested is None else status.FAILURE

        checkNoMove = CheckNoMove("CheckNoMove")

        class CheckArmSafe(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)

            def update(self):
                return status.SUCCESS if arm_is_safe() else status.FAILURE

        checkArmSafe = CheckArmSafe("CheckArmSafe")

        class MoveArmSafe(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)

            def update(self):
                if not cancel_navigation():
                    return status.RUNNING
                command_turn()
                via = mission.lift_arm_angles if mission._holding_cube else None
                return move_arm(mission.safe_arm_angles, via=via)

        moveArmSafe = MoveArmSafe("MoveArmSafe")

        class CheckUndocked(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)

            def update(self):
                if mission._undock_send_future is not None:
                    return status.FAILURE
                if fresh(mission._dock_status):
                    mission._undocked = not mission._dock_status.is_docked
                return status.SUCCESS if mission._undocked else status.FAILURE

        checkUndocked = CheckUndocked("CheckUndocked")

        class Undock(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)

            def update(self):
                if not cancel_navigation():
                    return status.RUNNING
                return mission.undock_robot()

        undock = Undock("Undock")

        class CheckLocalized(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)

            def update(self):
                if mission._localized and robot_pose() is not None:
                    return status.SUCCESS
                return status.FAILURE

        checkLocalized = CheckLocalized("CheckLocalized")

        class ControlRelocalization(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)
                self.future = None
                self.last_stamp = None
                self.last_yaw = None
                self.rotation = 0.0
                self.stable = 0

            def update(self):
                if not cancel_navigation():
                    return status.RUNNING
                if mission.grade != 'a':
                    command_turn()
                    return status.SUCCESS if robot_pose() is not None else status.RUNNING
                if self.future is None:
                    command_turn()
                    if not mission._localization_client.service_is_ready():
                        return status.RUNNING
                    mission._localized = False
                    self.future = mission._localization_client.call_async(EmptyService.Request())
                    self.started = time.monotonic()
                    self.reset_time = None
                    self.last_stamp = None
                    self.last_yaw = None
                    self.rotation = 0.0
                    self.stable = 0
                    return status.RUNNING
                if time.monotonic() - self.started > 300.0:
                    command_turn()
                    mission.get_logger().error('AMCL global localization timed out.')
                    return status.FAILURE
                if not self.future.done():
                    return status.RUNNING
                self.future.result()
                if self.reset_time is None:
                    # Discard the old AMCL belief left behind by the relocation tool.
                    self.reset_time = seconds()
                    mission._amcl_pose = None

                odom = robot_pose('odom')
                if odom is None:
                    command_turn()
                    return status.RUNNING
                angle = yaw(odom.transform.rotation)
                if self.last_yaw is not None:
                    delta = angle - self.last_yaw
                    self.rotation += abs(math.atan2(math.sin(delta), math.cos(delta)))
                self.last_yaw = angle

                pose = mission._amcl_pose
                if pose is not None:
                    stamp = Time.from_msg(pose.header.stamp).nanoseconds / 1e9
                    if stamp > self.reset_time and stamp != self.last_stamp:
                        self.last_stamp = stamp
                        covariance = pose.pose.covariance
                        converged = (0.0 <= covariance[0] < 0.01
                                     and 0.0 <= covariance[7] < 0.01
                                     and 0.0 <= covariance[35] < 0.03)
                        self.stable = self.stable + 1 if converged else 0
                if (self.rotation >= 2.0 * math.pi and self.stable >= 5
                        and fresh(pose) and robot_pose() is not None):
                    command_turn()
                    mission._localized = True
                    self.future = None
                    mission.get_logger().info('AMCL global localization completed.')
                    return status.SUCCESS
                command_turn(0.25)
                return status.RUNNING

        controlRelocalization = ControlRelocalization("ControlRelocalization")

        class CheckDelivered(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)

            def update(self):
                return status.SUCCESS if mission._cube_delivered else status.FAILURE

        checkDelivered = CheckDelivered("CheckDelivered")

        class CheckHolding(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)

            def update(self):
                return status.SUCCESS if mission._holding_cube else status.FAILURE

        checkHolding = CheckHolding("CheckHolding")

        class CheckAtGraspPoint(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)

            def update(self):
                return status.SUCCESS if at_pose(pick_pose) else status.FAILURE

        checkAtGraspPoint = CheckAtGraspPoint("CheckAtGraspPoint")

        class MovetoGraspPoint(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)

            def update(self):
                return move_to(pick_pose)

        movetoGraspPoint = MovetoGraspPoint("MovetoGraspPoint")

        class GraspPackage(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)
                self.started = None

            def update(self):
                if self.started is None:
                    command_turn()
                    result = move_arm(mission.pick_arm_angles, via=mission.lift_arm_angles)
                    if result != status.SUCCESS:
                        return result
                    mission._attach_pub.publish(Empty())
                    self.started = seconds()
                    return status.RUNNING
                if seconds() - self.started < mission._vacuum_wait_secs:
                    return status.RUNNING
                # The supplied bridge has commands only, no ROS attachment feedback.
                mission._holding_cube = True
                return status.SUCCESS

        graspPackage = GraspPackage("GraspPackage")

        class CheckAtDropPoint(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)

            def update(self):
                return status.SUCCESS if at_pose(drop_pose) else status.FAILURE

        checkAtDropPoint = CheckAtDropPoint("CheckAtDropPoint")

        class MovetoDrop(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)

            def update(self):
                return move_to(drop_pose)

        movetoDrop = MovetoDrop("MovetoDrop")

        class DropPackage(py_trees.behaviour.Behaviour):
            def __init__(self,name):
                super().__init__(name)
                self.started = None

            def update(self):
                if self.started is None:
                    command_turn()
                    result = move_arm(mission.place_arm_angles, via=mission.lift_arm_angles)
                    if result != status.SUCCESS:
                        return result
                    mission._detach_pub.publish(Empty())
                    self.started = seconds()
                    return status.RUNNING
                if seconds() - self.started < mission._vacuum_wait_secs:
                    return status.RUNNING
                # Commit both task flags together after the release settling time.
                mission._holding_cube = False
                mission._cube_delivered = True
                return status.SUCCESS

        dropPackage = DropPackage("DropPackage")

        vehicleOK = py_trees.composites.Selector(name = "vehicleOK", memory=False)
        SafefromOllision = py_trees.composites.Selector(name = "SafefromOllision", memory=False)
        MovePreparation = py_trees.composites.Selector(name = "MovePreparation", memory=False)
        ArmSafe = py_trees.composites.Selector(name = "ArmSafe", memory=False)
        RobotUndocked = py_trees.composites.Selector(name = "RobotUndocked", memory=False)
        PackageatDestination = py_trees.composites.Selector(name = "PackageatDestination", memory=False)
        HoldingPackage = py_trees.composites.Selector(name = "HoldingPackage", memory=False)
        atGraspPoint = py_trees.composites.Selector(name = "atGraspPoint", memory=False)
        atDropPoint = py_trees.composites.Selector(name = "atDropPoint", memory=False)
        root = py_trees.composites.Sequence(name="root", memory=False)
        prepareMove = py_trees.composites.Sequence(name="prepareMove", memory=False)
        dropthepackageornot = py_trees.composites.Sequence(name="dropthepackageornot", memory=True)
        graspthepackageornot = py_trees.composites.Sequence(name="graspthepackageornot", memory=True)

        # layer 1
        root.add_child(SafefromOllision)
        root.add_child(MovePreparation)
        root.add_child(PackageatDestination)

        # layer 2
        SafefromOllision.add_child(checkSafe)
        SafefromOllision.add_child(aovidCollision)
        MovePreparation.add_child(checkNoMove)
        MovePreparation.add_child(prepareMove)
        PackageatDestination.add_child(checkDelivered)
        PackageatDestination.add_child(dropthepackageornot)

        # layer 3
        prepareMove.add_child(ArmSafe)
        prepareMove.add_child(RobotUndocked)
        prepareMove.add_child(vehicleOK)
        dropthepackageornot.add_child(HoldingPackage)
        dropthepackageornot.add_child(atDropPoint)
        dropthepackageornot.add_child(dropPackage)

        # layer 4
        ArmSafe.add_child(checkArmSafe)
        ArmSafe.add_child(moveArmSafe)
        RobotUndocked.add_child(checkUndocked)
        RobotUndocked.add_child(undock)
        vehicleOK.add_child(checkLocalized)
        vehicleOK.add_child(controlRelocalization)
        HoldingPackage.add_child(checkHolding)
        HoldingPackage.add_child(graspthepackageornot)
        atDropPoint.add_child(checkAtDropPoint)
        atDropPoint.add_child(movetoDrop)

        # layer 5
        graspthepackageornot.add_child(atGraspPoint)
        graspthepackageornot.add_child(graspPackage)

        # layer 6
        atGraspPoint.add_child(checkAtGraspPoint)
        atGraspPoint.add_child(movetoGraspPoint)

        tree = py_trees.trees.BehaviourTree(root)
        tree.setup()
        next_tick = time.monotonic()
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)
                if time.monotonic() < next_tick:
                    continue
                next_tick = time.monotonic() + 0.1
                tree.tick()
                if root.status == status.SUCCESS:
                    self.get_logger().info('Pick-and-place command sequence completed.')
                    break
                if root.status == status.FAILURE:
                    raise RuntimeError('The mission behaviour tree failed.')
        finally:
            root.stop(status.INVALID)
            command_turn()
            # Poll late acceptance too, and wait for terminal action results.
            pending = [[f, None] for f in (self._nav_send_future,
                       self._arm_send_future, self._undock_send_future) if f is not None]
            deadline = time.monotonic() + 5.0
            while pending and rclpy.ok() and time.monotonic() < deadline:
                for item in pending[:]:
                    future, result = item
                    if not future.done():
                        continue
                    if future.exception() is not None or not future.result().accepted:
                        pending.remove(item)
                        continue
                    if result is None:
                        handle = future.result()
                        result = item[1] = handle.get_result_async()
                        if not result.done():
                            handle.cancel_goal_async()
                    if result.done():
                        pending.remove(item)
                if pending:
                    rclpy.spin_once(self, timeout_sec=0.05)
            if pending:
                self.get_logger().warn('Action cancellation was not confirmed before shutdown.')
            command_turn()


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