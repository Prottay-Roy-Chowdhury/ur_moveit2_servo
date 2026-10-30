from setuptools import setup, find_packages
from glob import glob

package_name = "ur_servo_vision"

setup(
    name=package_name,
    version="0.0.1",

    # Include ur_servo_vision and ur_servo_vision.nodes
    packages=find_packages(exclude=["test"]),

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
            "share/" + package_name + "/launch",
            glob("launch/*.py"),
        ),
        (
            "share/" + package_name + "/models",
            glob("models/*"),
        ),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml"),
        ),
    ],

    install_requires=["setuptools"],

    zip_safe=True,

    maintainer="prottay",
    maintainer_email="you@example.com",

    description="MediaPipe vision teleoperation for UR MoveIt Servo.",

    license="Apache-2.0",

    entry_points={
        "console_scripts": [
            "hand_to_twist = ur_servo_vision.nodes.hand_to_twist:main",
            "hand_to_twist_refined = ur_servo_vision.nodes.hand_to_twist_refined:main",
            "hand_to_twist_fixed = ur_servo_vision.nodes.hand_to_twist_fixed:main",
        ],
    },
)