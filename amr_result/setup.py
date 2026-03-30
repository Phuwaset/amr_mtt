from setuptools import find_packages, setup

package_name = 'amr_result'

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
    description='AMR MTT — Experimental Results & Analysis Tools',
    license='Academic',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'plot_odometry = amr_result.plot_odometry:main',
            'odom_plot = amr_result.odom_plot:main',
            'move_pose = amr_result.move_pose:main',
            'nav2_pose = amr_result.nav2_pose:main',
            'square_test = amr_result.square_test:main',
            'obstacle_avoidance_test = amr_result.obstacle_avoidance_test:main',
            'compare_obstacle_tests = amr_result.compare_obstacle_tests:main',
            'payload_test = amr_result.payload_test:main',
            'auto_system_test = amr_result.auto_system_test:main',
        ],
    },
)
