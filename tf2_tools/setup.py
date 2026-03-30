from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_mtt_tf2_tools'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Phuwaset Sibta',
    maintainer_email='s6703016411302@email.kmutnb.ac.th',
    description='TF2 / URDF inspection tools for AMR-MTT',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'urdf_tree = amr_mtt_tf2_tools.urdf_tree:main',
        ],
    },
)
