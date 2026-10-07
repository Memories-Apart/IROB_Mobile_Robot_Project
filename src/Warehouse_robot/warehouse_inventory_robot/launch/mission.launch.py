import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_inventory_robot')

    # Selects the grade this run is configured for; see simulation.launch.py
    # for what it changes in the world and in odometry. Here it decides who
    # publishes map -> odom, and which box the mission should treat as the drop
    # target.
    # Defaults to the GRADE environment variable so that one spelling works
    # everywhere: `GRADE=c pixi run mission` and, inside a pixi shell,
    # `GRADE=c ros2 launch ...`. An explicit grade:=c on the command line still
    # wins over it.
    grade_arg = DeclareLaunchArgument(
        'grade', default_value=EnvironmentVariable('GRADE', default_value='e'),
        choices=['e', 'c', 'a'],
        description='Which grade to configure the mission for (default: $GRADE, or e)')
    grade = LaunchConfiguration('grade')
    grade_is_a = PythonExpression(["'", grade, "' == 'a'"])

    headless_arg = DeclareLaunchArgument(
        'headless', default_value=EnvironmentVariable('HEADLESS', default_value='false'),
        choices=['true', 'false'],
        description='Run Gazebo headless (server only); RViz still shows the scene')

    # Must match the world simulation.launch.py selects, AND the name declared
    # inside that .sdf. relocate_robot teleports through
    # /world/<name>/set_pose, so a mismatch here means every click is silently
    # ignored.
    world = PythonExpression(
        ["'warehouse_dynamic' if '", grade, "' in ('c', 'a') else 'warehouse'"])

    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')

    # Simulation Layer
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('warehouse_inventory_robot'),
                'launch', 'simulation.launch.py'
            ])
        ]),
        launch_arguments={
            'grade': grade,
            'x': LaunchConfiguration('x_pose'),
            'y': LaunchConfiguration('y_pose'),
            'headless': LaunchConfiguration('headless'),
        }.items()
    )

    # E and C only. Their odometry is Gazebo's exact pose.
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(grade_is_a),
    )

    # Examiner tool, A grade only. Lets the robot be placed anywhere on the map
    # by clicking in RViz with the "Publish Point" tool.
    relocate_robot = Node(
        package='warehouse_inventory_robot',
        executable='relocate_robot',
        name='relocate_robot',
        output='screen',
        parameters=[{'world': world}],
        condition=IfCondition(grade_is_a),
    )

    # The arm controller's type comes from config/arm_controller_types.yaml,
    # loaded into controller_manager by gz_ros2_control at construction, so it is
    # already defined by the time this spawner runs.
    arm_traj_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "lite6_traj_controller", 
            "-c", "/controller_manager",
            "--param-file", os.path.join(pkg_share, 'config', 'arm_controllers.yaml')
        ],
        output="screen",
    )

    # Bringup is gated on observed readiness, not on fixed delays. Wall-clock
    # delays are machine-dependent.
    wait_for_sim = Node(
        package='warehouse_inventory_robot',
        executable='wait_for_ready',
        name='wait_for_sim',
        output='screen',
        arguments=['--label', 'sim',
                   '--node', 'controller_manager',
                   '--call-service', '/controller_manager/list_controllers',
                   '--service', '/controller_manager/set_parameters',
                   '--topic', '/scan',
                   '--timeout', '300'],
    )

    # TODO L1 [E/C/A]: Launch and configure the existing Nav2 navigation stack.
    # The student mission uses a behavior tree already in E; Nav2's internal BT
    # does not replace that mission tree. Keep the two-terminal workflow unless
    # intentionally changing it, so mission_node is not launched twice.
    #
    # Per Q&A 3: the default nav2 config uses MPPIController which fails near
    # obstacles (e.g. the shelf). We use RewrittenYaml to patch only the
    # FollowPath plugin on top of the original turtlebot4 nav2.yaml, so all
    # other parameters (costmap, docking, collision monitor, etc.) remain intact.
    from nav2_common.launch import RewrittenYaml
    nav2_params = RewrittenYaml(
        source_file=get_package_share_directory('turtlebot4_navigation') + '/config/nav2.yaml',
        param_rewrites={
            'controller_server.ros__parameters.FollowPath.plugin':
                'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            'controller_server.ros__parameters.FollowPath.desired_linear_vel': '0.26',
            'controller_server.ros__parameters.FollowPath.lookahead_dist': '0.4',
            'controller_server.ros__parameters.FollowPath.rotate_to_heading_angular_vel': '1.0',
            'controller_server.ros__parameters.FollowPath.regulated_linear_scaling_min_speed': '0.1',
            'controller_server.ros__parameters.FollowPath.cost_scaling_dist': '0.4',
            'controller_server.ros__parameters.FollowPath.inflation_cost_scaling_factor': '4.0',
        },
        convert_types=True,
    )
    nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('turtlebot4_navigation'),
            'launch',
            'nav2.launch.py',
        )
    ),
    launch_arguments={
        'use_sim_time': 'true',
        'params_file': nav2_params,
    }.items(),
)

    # TODO L2 [A only]: Launch and configure AMCL. The other grades get map -> odom from the static publisher
    # above, which is exact. Remember to launch amcl only for A grade.

    # TODO L3 [E/C/A]: Launch the map server with the provided warehouse map.
    # NOTE: We provide a map at src/Warehouse_robot/warehouse_inventory_robot/maps
    map_server = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    parameters=[{
        'use_sim_time': True,
        'yaml_filename': os.path.join(
            pkg_share, 'maps', 'warehouse.yaml'
        ),
    }],
)

    # TODO L4 [E/C/A]: Activate lifecycle nodes and check readiness before motion.
    # Add the new launch actions to LaunchDescription or the appropriate event
    # handler, pass configuration/use_sim_time, and avoid duplicate map/TF nodes.
    # For A, additionally check AMCL readiness; no localization rewrite is needed.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }],
    )


    #
    # A fixed delay is fine for ordering things. It cannot fix one failure you
    # may hit if you use nav2_lifecycle_manager, though, so it is worth knowing
    # about in advance: it gives its lifecycle service calls a hardcoded
    # deadline, and on a cold start (Gazebo still loading meshes) map_server's
    # reply can miss it. The manager then never sends ACTIVATE and map_server
    # stays `inactive` indefinitely -- no /map, an empty RViz, and the costmaps
    # complaining "Can't update static costmap layer". Waiting longer does not
    # help once the manager has given up, so if you see that, check the state
    # and finish the transition yourself:
    #
    #     ros2 lifecycle get /map_server
    #     ros2 lifecycle set /map_server activate

    return LaunchDescription([
        grade_arg, x_pose_arg, y_pose_arg, headless_arg,

        simulation_launch,

        relocate_robot,

        # Start watching for the simulation to come up.
        wait_for_sim,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_sim,
                on_exit=[
                    arm_traj_spawner,
                    static_map_to_odom,
                    map_server,
                    lifecycle_manager,
                    nav2_launch,
                ],
            )
        ),
    ])
