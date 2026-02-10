from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # นิยามการทำงานของ Node teleop_twist_keyboard
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard_node',
        output='screen',
        prefix='xterm -e', # สำคัญ: ใช้เปิดหน้าต่างใหม่เพื่อให้รับค่าจาก keyboard ได้
        # No remapping needed - publish directly to /cmd_vel
    )

    return LaunchDescription([
        teleop_node
    ])