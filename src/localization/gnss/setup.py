from setuptools import find_packages, setup

package_name = 'gnss'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gnss_bringup.launch.py']),
        ('share/' + package_name + '/config',
            [
                'config/ntrip_client.yaml',
                'config/gnss_driver.yaml',
            ]
        )
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neha',
    maintainer_email='neha@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rtk_corrections_node = gnss.rtk_corrections_node:main',
            'odom_to_tf_publisher = gnss.odom_to_tf_publisher:main',
            'gnss_interface = gnss.gnss_interface:main',
        ],
    },
)
