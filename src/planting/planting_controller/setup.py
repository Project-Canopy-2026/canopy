from setuptools import find_packages, setup

package_name = 'planting_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/planting_bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alina',
    maintainer_email='alina@todo.todo',
    description='Planting controller — serial bridge, FSM, and manual tester nodes',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'serial_bridge = planting_controller.serial_bridge:main',
            'planting_fsm = planting_controller.planting_fsm:main',
            'manual_fsm_tester = planting_controller.manual_fsm_tester:main',
        ],
    },
)
