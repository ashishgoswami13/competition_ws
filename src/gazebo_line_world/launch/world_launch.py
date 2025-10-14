#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package path
    pkg_path = get_package_share_directory('gazebo_line_world')
    
    # Path to the world file
    world_path = os.path.join(pkg_path, 'worlds', 'line_follower.world')
    
    # Path to the models directory
    models_path = os.path.join(pkg_path, 'models')
    
    # Get existing GAZEBO_MODEL_PATH and append our models path
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if existing_model_path:
        full_model_path = f"{models_path}:{existing_model_path}"
    else:
        full_model_path = models_path
    
    return LaunchDescription([
        # Set environment variables for software rendering and Qt
        SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),
        SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb'),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=full_model_path),
        
        # Launch Gazebo with the world file
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path],
            output='screen',
            shell=False
        ),
    ])
