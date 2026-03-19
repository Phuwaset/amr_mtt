from setuptools import find_packages, setup

package_name = 'amr_mtt_task_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sphuwaset_ros',
    maintainer_email='100694995+Phuwaset@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coordinator_node = amr_mtt_task_planner.coordinator_node:main',
            'planning_scene_setup = amr_mtt_task_planner.planning_scene_setup:main',
            'nav_sequence = amr_mtt_task_planner.nav_sequence:main',
            'save_waypoint = amr_mtt_task_planner.save_waypoint:main',
            'task_sequence = amr_mtt_task_planner.task_sequence:main',
            'set_initial_pose = amr_mtt_task_planner.set_initial_pose:main',
            'pick_ik = amr_mtt_task_planner.pick_ik:main',
            'arm_jog_node = amr_mtt_task_planner.arm_jog_node:main',
            'nav_waypoint_server = amr_mtt_task_planner.nav_waypoint_server:main',
            'map_manager_node = amr_mtt_task_planner.map_manager_node:main',
        ],
    },
)
