from setuptools import setup, find_packages 
import os
from glob import glob

package_name = 'distance_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 告诉ROS 2安装我们的掩码文件
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        # 包含launch文件夹下的所有.launch.py文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 包含config文件夹下的所有.yaml文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name', # 替换为你的名字
    maintainer_email='your_email@example.com', # 替换为你的邮箱
    description='ROS 2 node to detect objects in a masked proximity zone.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 将'detector_node'注册为可执行文件
            'distance_node = distance_detector.distance_node:main',
            'distance_node_cv = distance_detector.distance_node_cv:main',
        ],
    },
)