from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fast_drone_ctrl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index 등록 (colcon build 필수)
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일 설치
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kj',
    maintainer_email='kj@todo.com',
    description='고속 ISR 드론 PX4 Offboard 제어',
    license='MIT',
    entry_points={
        'console_scripts': [
            # ros2 run fast_drone_ctrl offboard_node
            'offboard_node = fast_drone_ctrl.offboard_node:main',
        ],
    },
)
