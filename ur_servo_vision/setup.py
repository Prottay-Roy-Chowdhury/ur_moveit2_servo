from setuptools import setup
from glob import glob

package_name = "ur_servo_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/models", glob("models/*")),
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
        ],
    },
)