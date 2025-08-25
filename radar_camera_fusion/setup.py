from setuptools import setup
from glob import glob
import os

package_name = 'radar_camera_fusion'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='astrabeam',
    maintainer_email='kohmacht@astrabeam.com',
    description='Overlay radar point cloud onto camera image',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fusion_node = radar_camera_fusion.fusion_node:main',
            'fusion_node_debug = radar_camera_fusion.fusion_node_debug:main'
        ],
    },
)


