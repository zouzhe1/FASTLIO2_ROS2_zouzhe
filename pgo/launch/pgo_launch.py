import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def profile_owner(profile):
    if profile != 'mapping':
        raise RuntimeError('PGO launch requires operational_profile=mapping')
    return 'pgo'


def _validate_profile(context):
    profile_owner(LaunchConfiguration('operational_profile').perform(context))
    return []


def generate_launch_description():
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("pgo"), "rviz", "pgo.rviz"]
    )
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo.yaml"]
    )

    lio_config_path = PathJoinSubstitution(
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
                    {"config_path": lio_config_path.perform(launch.LaunchContext())},
                    {"operational_profile": LaunchConfiguration('operational_profile')},
                ]
            ),
            launch_ros.actions.Node(
                package="pgo",
                namespace="pgo",
                executable="pgo_node",
                name="pgo_node",
                output="screen",
                parameters=[
                    {"config_path": pgo_config_path.perform(launch.LaunchContext())},
                    {"operational_profile": LaunchConfiguration('operational_profile')},
                    {"publish_global_tf": True},
                ]
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="pgo",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            )
        ]
    )
