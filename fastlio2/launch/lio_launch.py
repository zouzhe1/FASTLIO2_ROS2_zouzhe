import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def profile_owner(profile):
    if profile not in ('mapping', 'localization'):
        raise RuntimeError('lio_node is online only in mapping or localization profile')
    return 'none'


def _validate_profile(context):
    profile_owner(LaunchConfiguration('operational_profile').perform(context))
    return []


def generate_launch_description():

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "rviz", "fastlio2.rviz"]
    )

    config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )


    return launch.LaunchDescription(
        [
            DeclareLaunchArgument('operational_profile', default_value='mapping'),
            OpaqueFunction(function=_validate_profile),
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[
                    {"config_path": config_path.perform(launch.LaunchContext())},
                    {"operational_profile": LaunchConfiguration('operational_profile')},
                ]
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="fastlio2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            ),
        ]
    )
