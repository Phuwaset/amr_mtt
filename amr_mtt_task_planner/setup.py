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
            'coordinator_demo_tf = amr_mtt_task_planner.coordinator_demo_tf:main',
            'spawn_box_marker = amr_mtt_task_planner.spawn_box_marker:main',
            'planning_scene_setup = amr_mtt_task_planner.planning_scene_setup:main',
            'pick_sequence = amr_mtt_task_planner.pick_sequence:main',
            'nav_sequence = amr_mtt_task_planner.nav_sequence:main',
            'save_waypoint = amr_mtt_task_planner.save_waypoint:main',
            'task_sequence = amr_mtt_task_planner.task_sequence:main',
            'set_initial_pose = amr_mtt_task_planner.set_initial_pose:main',
            'pick_drop_sequence = amr_mtt_task_planner.pick_drop_sequence:main',
        ],
    },
)
