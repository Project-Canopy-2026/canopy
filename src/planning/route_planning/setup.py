from setuptools import find_packages, setup

package_name = "route_planning"

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
    maintainer="wheitman",
    maintainer_email="will@heit.mn",  # See package.xml
    description="TODO: Package description",  # See package.xml
    license="TODO: License declaration",  # See package.xml
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "route_planner = route_planning.route_planner:main",
            "demo_waypoint_follower = route_planning.demo_waypoint_follower:main",
        ],
    },
)
