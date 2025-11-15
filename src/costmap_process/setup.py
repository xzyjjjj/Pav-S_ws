from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'costmap_process'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aic',
    maintainer_email='xuzhenjyu23@mails.ucas.ac.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 'costmap_num_node = costmap_num.costmap_selector:main',
            # 'costmap_stage_node = costmap_stage.costmap_stage:main',
            'topic_saver_node = scripts.topic_saver:main',
            'senamic_node = new.combined:main',
            'costmap_pub_node = costmap_pub.costmap_pub:main',
            'map_vis_publisher = map_vis.map_vis_publisher:main',
            'costmap_vis_publisher = map_vis.costmap_vis_publisher:main'
        ],
    },
)
