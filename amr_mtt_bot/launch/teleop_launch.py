from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Path to your custom teleop script
    # Points to the script we created: ~/amr_mtt/src/amr_mtt/amr_mtt_bot/amr_mtt_bot/scripts/amr_teleop.py
    # Note: Using absolute path for convenience, better practice is to install via package setup.py later.
    script_path = os.path.expanduser('~/amr_mtt/src/amr_mtt/amr_mtt_bot/amr_mtt_bot/scripts/amr_teleop.py')

    # Execute custom python script in a new terminal window
    teleop_cmd = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'python3', script_path],
        output='screen'
    )

    return LaunchDescription([
        teleop_cmd
    ])