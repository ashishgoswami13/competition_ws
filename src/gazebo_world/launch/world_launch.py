<?xml version="1.0"?>
<launch>
    <!-- Set environment variables for software rendering and Qt -->
    <env name="LIBGL_ALWAYS_SOFTWARE" value="1"/>
    <env name="QT_QPA_PLATFORM" value="xcb"/>
    
    <!-- Set GAZEBO_MODEL_PATH -->
    <env name="GAZEBO_MODEL_PATH" value="$(find gazebo_line_world)/models"/>

    <!-- Launch Gazebo with the world file -->
    <node name="gazebo" pkg="gazebo_ros" type="gazebo" args="--verbose $(find gazebo_line_world)/worlds/line_follower.world" output="screen"/>
</launch>

