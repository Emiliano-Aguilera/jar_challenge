"""Temporary controlled-test launch. Competition launch stays unchanged."""
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share=get_package_share_directory('equipo_jar')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('victimas',default_value='2'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','navegacion.launch.py')),
            launch_arguments={'use_sim_time':LaunchConfiguration('use_sim_time'),'localizacion':'true'}.items()),
        ExecuteProcess(cmd=[sys.executable,os.path.join(share,'senuelos_control','control.py'),
                            '--ros-args','-p',['use_sim_time:=',LaunchConfiguration('use_sim_time')]],output='screen'),
    ])
