from setuptools import find_packages, setup

package_name = 'planting_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        'planting_controller',
        'unit_test',
        'unit_test.can_tests',
    ],
    package_dir={
        'planting_controller': 'planting_controller',
        'unit_test': '../unit_test',
        'unit_test.can_tests': '../unit_test/can_tests',
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/planting_bringup.launch.py',
            'launch/linak_test.launch.py',
        ]),
        ('share/' + package_name + '/eds', [
            'planting_controller/LINAK-actuator-v3-1.eds',
        ]),
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
            'linak_can_node = planting_controller.linak_can_node:main',
            'linak_test = unit_test.can_tests.linak_cmd_test_node:main',
            'planting_fsm_outmax = planting_controller.planting_fsm_outmax:main',
            'planting_fsm_test = planting_controller.planting_fsm_test_node:main',
        ],
    },
)
