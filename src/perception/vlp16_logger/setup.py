from setuptools import find_packages, setup

package_name = 'vlp16_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'vlp16_publisher = vlp16_logger.vlp16_publisher:main',
            'vlp16_subscriber = vlp16_logger.vlp16_subscriber:main'
        ],
    },
)
