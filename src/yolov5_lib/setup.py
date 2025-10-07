# from setuptools import find_packages, setup

# package_name = 'yolov5_lib'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='kolp',
#     maintainer_email='kolp@todo.todo',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )

import os
from glob import glob
from setuptools import setup, find_packages
package_name = 'yolov5_lib'

setup(
    name=package_name,
    version='0.0.0',
    # 关键改动1: 告诉setuptools在'yolov5_lib'子文件夹里寻找要打包的模块
    package_dir={'': 'yolov5_lib'},
    # 关键改动2: 明确指出models和utils是我们要打包的顶级模块
    packages=find_packages(where='yolov5_lib'),
    py_modules=['export'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('data/*.yaml')),
        (os.path.join('share', package_name, 'weights'), glob('weights/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@todo.todo',
    description='A ROS2 wrapper for YOLOv5 library modules.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)