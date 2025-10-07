from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_detector'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- 这是新增的部分 ---
        # 包含launch文件夹下的所有.launch.py文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 包含config文件夹下的所有.yaml文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    # install_requires=['setuptools', 'PyYAML', 'opencv-python', 'Pillow', 'torch', 'torchvision'],
    zip_safe=True,
    maintainer='kolp',
    maintainer_email='kolp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 格式： 'ros2 run时使用的命令名 = Python 包名.Python 文件名:入口函数名'
            'yolo_node = yolo_detector.yolo_node:main',
            'cmd_vel_limiter = yolo_detector.cmd_vel_limiter:main',
        ],
    },
)
