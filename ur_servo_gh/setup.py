from setuptools import setup, find_packages

package_name = "ur_servo_gh"

setup(
    name=package_name,
    version="0.0.1",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name + "/config",
            ["config/gh_direction_to_twist.yaml"],
        ),
    ],

    install_requires=["setuptools"],

    zip_safe=True,

    maintainer="prottay",
    maintainer_email="you@example.com",

    description=(
        "Grasshopper directional teleoperation "
        "for UR MoveIt Servo."
    ),

    license="Apache-2.0",

    entry_points={
        "console_scripts": [
            (
                "gh_direction_to_twist = "
                "ur_servo_gh.nodes.gh_direction_to_twist:main"
            ),
        ],
    },
)