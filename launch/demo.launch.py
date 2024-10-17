#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
import launch_ros
from launch_ros.actions import SetParameter, Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time',  default='true')    
    use_simulator = LaunchConfiguration('use_sim_time',  default='true')
    print(use_sim_time)
    this_directory = get_package_share_directory('stage_ros2')
    launch_dir = os.path.join(this_directory, 'launch')

    enforce_prefixes = LaunchConfiguration('enforce_prefixes')
    enforce_prefixes_cmd = DeclareLaunchArgument(
        'enforce_prefixes',
        default_value='false',
        description='on true a prefixes are used for a single robot environment')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    
    one_tf_tree = LaunchConfiguration('one_tf_tree')
    one_tf_tree_cmd = DeclareLaunchArgument(
        'one_tf_tree',
        default_value='false',
        description='on true all tfs are published with a namespace on /tf and /tf_static')
   
    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    stage = LaunchConfiguration('stage')
    declare_stage_cmd = DeclareLaunchArgument(
        'stage',
        default_value='True',
        description='Whether run a stage')

    rviz = LaunchConfiguration('rviz')
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Whether run a rviz')

    world = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument(
        'world', default_value='cave_one_robot',
        description='world to load in stage and rviz config [cave, example]')
        
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s'],
        output='screen',
        condition=IfCondition(use_simulator)
    )
          
    return LaunchDescription([
        declare_namespace_cmd,
        SetParameter(name='use_sim_time', value=True),
        SetParameter(name='use_simulator', value=True),
        declare_rviz_cmd,
        gazebo_server,
        declare_stage_cmd,
        enforce_prefixes_cmd,
        one_tf_tree_cmd,
        declare_world,
        declare_use_sim_time_cmd,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
            condition=IfCondition(rviz),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'config': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'stage.launch.py')),
            condition=IfCondition(stage),
            launch_arguments={'one_tf_tree':one_tf_tree,
                              'enforce_prefixes':enforce_prefixes,
                              'world': world}.items()),
    ])
