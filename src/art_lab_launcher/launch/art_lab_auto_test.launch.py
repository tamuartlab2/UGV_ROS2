from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    robotDescription = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('robot_description')),
         '/robot_description.launch.py'])
      )

    teensySensors = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('teensy_sensors')),
         '/Teensy.launch.py'])
      )

    roboclaw = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ros2_roboclaw')),
         '/ros2_roboclaw.launch.py'])
      ) 

    lidar = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('sllidar_ros2'), 'launch'),
         '/sllidar_a1_launch.py'])
      ) 
      
    
    lidar_s2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('sllidar_ros2'), 'launch'),
         '/sllidar_s2_launch.py'])
      )  
    
    #K. Lee's code
    realsense_cam = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('realsense_ros2_camera'), 'launch'),
         '/ros2_intel_realsense.launch.py'])
      ) 
    
    # Realsense official code
    #realsense_cam = IncludeLaunchDescription(
    #  PythonLaunchDescriptionSource([os.path.join(
    #     get_package_share_directory('realsense2_camera'), 'launch'),
    #     '/rs_launch.py'])
    #  ) 
    
    Reach_M2 = Node(
                   package="Reach_Device",
                   executable="ReachM2"
               )
               
    compressed_image = Node(
                   package="image_processing",
                   executable="compressed_image"
               )
    
    web_socket = IncludeLaunchDescription(
      XMLLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rosbridge_server'), 'launch'),
         '/rosbridge_websocket_launch.xml'])
      ) 
    
    localization_ekf = Node(
                   package="loca_and_nav",
                   executable="localization",
                   output='screen',
                   emulate_tty=True
               )
    
    nav_obstacle_force = Node(
                   package="loca_and_nav",
                   executable="obstacle_force",
                   output='screen',
                   emulate_tty=True
               )
    
    nav_speed_pub = Node(
                   package="loca_and_nav",
                   executable="speed_pub",
                   output='screen',
                   emulate_tty=True
               )
    
    linear_force = Node(
                package="loca_and_nav",
                executable="linear_force",
                output='screen',
                emulate_tty=True
              )
    
    spring_damper = Node(
                package="loca_and_nav",
                executable="spring_damper",
                output='screen',
                emulate_tty=True
              )
    
    bag_record = Node(
                package="art_lab_launcher",
                executable="bag_recorder",
                output='screen',
                emulate_tty=True
              )
     
    
    return LaunchDescription([ 
        robotDescription,
        teensySensors,
        roboclaw,
        # lidar,
        lidar_s2,
        realsense_cam,
        Reach_M2,
        localization_ekf,
        web_socket,
        nav_obstacle_force,
        nav_speed_pub,
        linear_force,
        spring_damper,
	      compressed_image,
        bag_record
    ])
