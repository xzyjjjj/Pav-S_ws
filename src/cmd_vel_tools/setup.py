from setuptools import setup, find_packages

package_name = 'cmd_vel_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'red_segment_msg'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Velocity limiter and odom distance tracker nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_limiter = cmd_vel_tools.cmd_vel_limiter:main',
            'obstacle_analyzer = cmd_vel_tools.obstacle_analyzer:main',
            'debug_node = cmd_vel_tools.debug_node:main',
            'red_segment = cmd_vel_tools.red_segment:main',
        ],
    },
)
