from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'controller_manager': {
                    'update_rate': 100,
                    'robotic_arm_controller': {
                        'type': 'controller/VisualServoingController'
                    },
                    'joint_state_broadcaster': {
                        'type': 'joint_state_broadcaster/JointStateBroadcaster',
                        'joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
                    }
                }}
            ]
        )
    ])
