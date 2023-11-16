from setuptools import find_packages, setup

package_name = 'lidar_with_mirror'

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
    maintainer='root',
    maintainer_email='yasuo.hayashibara@it-chiba.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'check_estimate_posture = lidar_with_mirror.check_estimate_posture:main',
            'check_ground_height = lidar_with_mirror.check_ground_height:main',
            'demo_change_angle = lidar_with_mirror.demo_change_angle:main',
            'estimate_posture = lidar_with_mirror.estimate_posture:main',
            'experiment = lidar_with_mirror.experiment:main',
            'merge_measured_data = lidar_with_mirror.merge_measured_data:main',
            'mirror_lidar = lidar_with_mirror.mirror_lidar:main',
        ],
    },
)
