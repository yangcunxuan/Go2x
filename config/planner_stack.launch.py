"""CMU-style planner stack for GO2 + MID360 + FAST-LIO2.

Replaces the Nav2 2D pipeline with terrain analysis + FAR planner + local
planner. All inputs are remapped onto the existing FAST-LIO topics:

  /Odometry               (nav_msgs/Odometry, camera_init <- body)
  /cloud_registered_body  (sensor_msgs/PointCloud2, body frame)

Modes:
  sensing - terrain_analysis + terrain_analysis_ext only. First power-on
            verification step: check /terrain_map in RViz, CPU and memory.
  full    - adds far_planner + local_planner (localPlanner + pathFollower).
            pathFollower publishes /cmd_vel; nav_motion_bridge must be
            reviewed before letting it reach the dog.

Usage:
  ros2 launch /project/config/planner_stack.launch.py mode:=sensing
  ros2 launch /project/config/planner_stack.launch.py mode:=full
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# FAST-LIO topics -> Go2_planner_suite topic names.
ODOM_REMAPS = [
    ('/lidar_odometry/pose', '/Odometry'),
    ('/lidar_odometry/deskewed_scan_points', '/cloud_registered_body'),
]

FAR_REMAPS = [
    ('/odom_world', '/Odometry'),
    ('/terrain_cloud', '/terrain_map_ext'),
    ('/terrain_local_cloud', '/terrain_map'),
    ('/scan_cloud', '/cloud_registered_body'),
]

FULL_MODE = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'full'"])

# GO2 standing dimensions: body ~0.70 x 0.35 m, back height ~0.40 m.
TERRAIN_PARAMS = {
    'scanVoxelSize': 0.05,
    'decayTime': 1.0,
    'noDecayDis': 4.0,
    'clearingDis': 1.0,
    'useSorting': True,
    'quantileZ': 0.2,
    'considerDrop': True,
    'limitGroundLift': True,
    'maxGroundLift': 0.15,
    'clearDyObs': True,
    'minDyObsDis': 0.3,
    'minDyObsAngle': 0.0,
    'minDyObsRelZ': -0.5,
    'absDyObsRelZThre': 0.2,
    'minDyObsVFOV': -16.0,
    'maxDyObsVFOV': 16.0,
    'minDyObsPointNum': 1,
    'noDataObstacle': False,
    'noDataBlockSkipNum': 0,
    'minBlockPointNum': 10,
    'vehicleHeight': 0.4,
    'voxelPointUpdateThre': 10,
    'voxelTimeUpdateThre': 2.0,
    'minRelZ': -0.5,
    'maxRelZ': 1.0,
    'disRatioZ': 0.1,
}

TERRAIN_EXT_PARAMS = {
    'scanVoxelSize': 0.05,
    'decayTime': 2.0,
    'noDecayDis': 8.0,
    'clearingDis': 1.0,
    'useSorting': False,
    'quantileZ': 0.25,
    'limitGroundLift': True,
    'maxGroundLift': 0.15,
    'clearDyObs': False,
    'minDyObsDis': 0.3,
    'minDyObsAngle': 0.0,
    'minDyObsRelZ': -0.5,
    'absDyObsRelZThre': 0.2,
    'minDyObsVFOV': -16.0,
    'maxDyObsVFOV': 16.0,
    'minDyObsPointNum': 1,
    'noDataObstacle': False,
    'noDataBlockSkipNum': 0,
    'minBlockPointNum': 10,
    'vehicleHeight': 0.4,
    'voxelPointUpdateThre': 10,
    'voxelTimeUpdateThre': 2.0,
    'minRelZ': -0.5,
    'maxRelZ': 1.0,
    'disRatioZ': 0.1,
    'terrainUnderVehicle': -0.1,
    'terrainConnThre': 0.3,
    'minTerrainRelZ': -0.5,
    'maxTerrainRelZ': 0.5,
    'minTerrainPointNum': 1,
}

LOCAL_PLANNER_PARAMS = {
    'vehicleLength': 0.7,
    'vehicleWidth': 0.35,
    'sensorOffsetX': 0.0,
    'sensorOffsetY': 0.0,
    'twoWayDrive': False,
    'laserVoxelSize': 0.1,
    'terrainVoxelSize': 0.1,
    'useTerrainAnalysis': True,
    'checkObstacle': True,
    'checkRotObstacle': False,
    'adjacentRange': 4.5,
    'obstacleHeightThre': 0.35,
    'groundHeightThre': 0.1,
    'costHeightThre': 0.1,
    'costScore': 0.05,
    'useCost': False,
    'pointPerPathThre': 3,
    'minRelZ': -0.2,
    'maxRelZ': 1.0,
    # Speeds start conservative (0.3 m/s) until reliability is proven; the
    'maxSpeed': 0.3,
    'dirWeight': 0.20,
    'dirThre': 120.0,
    'dirToVehicle': True,
    'pathScale': 1.25,
    'minPathScale': 0.75,
    'pathScaleStep': 0.25,
    'pathScaleBySpeed': True,
    'minPathRange': 1.0,
    'pathRangeStep': 0.5,
    'pathRangeBySpeed': False,
    'pathCropByGoal': True,
    'autonomyMode': True,
    'autonomySpeed': 0.3,
    'joyToSpeedDelay': 2.0,
    'joyToCheckObstacleDelay': 5.0,
    'goalClearRange': 0.5,
    'goalX': 0.0,
    'goalY': 0.0,
}

PATH_FOLLOWER_PARAMS = {
    'sensorOffsetX': 0.0,
    'sensorOffsetY': 0.0,
    'pubSkipNum': 1,
    'twoWayDrive': False,
    'lookAheadDis': 3.0,
    'yawRateGain': 2.5,
    'stopYawRateGain': 5.0,
    'maxYawRate': 46.0,
    'maxSpeed': 0.3,
    'maxAccel': 0.2,
    'switchTimeThre': 1.0,
    'dirDiffThre': 0.35,
    'stopDisThre': 0.2,
    'slowDwnDisThre': 1.0,
    'useInclRateToSlow': False,
    'inclRateThre': 120.0,
    'slowRate1': 0.25,
    'slowRate2': 0.5,
    'slowTime1': 2.0,
    'slowTime2': 2.0,
    'useInclToStop': False,
    'inclThre': 45.0,
    'stopTime': 5.0,
    'noRotAtStop': False,
    'noRotAtGoal': True,
    # Without autonomyMode the follower never converts paths into velocity.
    'autonomyMode': True,
    'autonomySpeed': 0.3,
    'joyToSpeedDelay': 2.0,
}


def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory

    local_params = dict(LOCAL_PLANNER_PARAMS)
    local_params['pathFolder'] = get_package_share_directory('local_planner') + '/paths'

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='sensing',
            description='sensing = terrain only; full = + far/local planner'),

        # terrain_analysis hardcodes frame_id "map" on its output clouds while
        # the coordinates are actually camera_init (FAST-LIO world). far_planner
        # then TF-looks-up camera_init <- map and fails. Alias them, same trick
        # the old Nav2 stack used for map_level -> map.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_camera_init_alias',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'map'],
        ),

        Node(
            package='terrain_analysis',
            executable='terrainAnalysis',
            name='terrainAnalysis',
            output='screen',
            parameters=[TERRAIN_PARAMS],
            remappings=ODOM_REMAPS,
        ),
        Node(
            package='terrain_analysis_ext',
            executable='terrainAnalysisExt',
            name='terrainAnalysisExt',
            output='screen',
            parameters=[TERRAIN_EXT_PARAMS],
            remappings=ODOM_REMAPS,
        ),
        Node(
            package='far_planner',
            executable='far_planner',
            name='far_planner',
            output='screen',
            parameters=['/project/config/far_planner_go2.yaml'],
            remappings=FAR_REMAPS,
            condition=IfCondition(FULL_MODE),
        ),
        Node(
            package='local_planner',
            executable='localPlanner',
            name='localPlanner',
            output='screen',
            parameters=[local_params],
            remappings=ODOM_REMAPS,
            condition=IfCondition(FULL_MODE),
        ),
        Node(
            package='local_planner',
            executable='pathFollower',
            name='pathFollower',
            output='screen',
            parameters=[PATH_FOLLOWER_PARAMS],
            remappings=ODOM_REMAPS,
            condition=IfCondition(FULL_MODE),
        ),
    ])
