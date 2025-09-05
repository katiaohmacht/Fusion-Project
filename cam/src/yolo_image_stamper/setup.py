from setuptools import find_packages, setup

package_name = 'yolo_image_stamper'

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
    maintainer='astrabeam3',
    maintainer_email='kohmacht@astrabeam.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_image_stamper = yolo_image_stamper.yolo_image_stamper:main'
        ],
    },
)
