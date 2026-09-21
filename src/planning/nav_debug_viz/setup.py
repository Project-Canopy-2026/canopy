from setuptools import find_packages, setup

package_name = "nav_debug_viz"

setup(
    name=package_name,
    version="0.0.0",  # See package.xml
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Shuyi Lin",
    maintainer_email="shuyilin@andrew.cmu.edu",  # See package.xml
    description="Debug visualization for localization and navigation",  # See package.xml
    license="MIT",  # See package.xml
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"nav_viz_node = {package_name}.nav_viz_node:main",
        ],
    },
)
