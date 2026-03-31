from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'canopy_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'canopy_bringup', 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', 'canopy_bringup', 'rviz'),
            glob('rviz/*.rviz')),
        (os.path.join('share', 'canopy_bringup', 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shuyi',
    maintainer_email='1798045064@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
