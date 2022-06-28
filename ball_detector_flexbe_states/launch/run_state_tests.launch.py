from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    flexbe_testing_dir = get_package_share_directory('flexbe_testing')
    ball_detector_states_test_dir = get_package_share_directory('ball_detector_flexbe_states')

    path = ball_detector_states_test_dir + "/test"

    testcases  = path + "/ball_detector_state.test \n"

    return LaunchDescription([
        DeclareLaunchArgument("pkg", default_value="ball_detector_flexbe_states"),
        DeclareLaunchArgument("compact_format", default_value="True"),
        DeclareLaunchArgument("testcases", default_value=testcases),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(flexbe_testing_dir + "/launch/flexbe_testing.launch.py"),
            launch_arguments={
                'package': LaunchConfiguration("pkg"),
                'compact_format': LaunchConfiguration("compact_format"),
                'testcases': LaunchConfiguration("testcases"),
            }.items()
        )
    ])
