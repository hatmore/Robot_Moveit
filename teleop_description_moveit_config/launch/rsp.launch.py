# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_rsp_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("lindenbot_teleop_description", package_name="teleop_description_moveit_config").to_moveit_configs()
#     return generate_rsp_launch(moveit_config)

from launch import LaunchDescription                                                                                                        
from launch_ros.actions import Node                                                                                                         
from moveit_configs_utils import MoveItConfigsBuilder                                                                                       
                                                                                                                                            
                                                                                                                                            
def generate_launch_description():                                                                                                          
    moveit_config = MoveItConfigsBuilder(                                                                                                   
        "lindenbot_teleop_description",                                                                                                     
        package_name="teleop_description_moveit_config"                                                                                     
    ).to_moveit_configs()                                                                                                                   
                                                                                                                                            
    return LaunchDescription([                                                                                                              
        Node(                                                                                                                               
            package="robot_state_publisher",                                                                                                
            executable="robot_state_publisher",                                                                                             
            output="screen",                                                                                                                
            parameters=[moveit_config.robot_description],                                                                                   
            remappings=[                                                                                                                    
                ("/joint_states", "/cerebellum_sdk/arm/joint_states"),                                                                      
            ],                                                                                                                              
        )                                                                                                                                   
    ]) 