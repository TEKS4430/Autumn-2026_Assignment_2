from setuptools import find_packages, setup
import os
from glob import glob

package_name = "sensing_assignment"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "resource"), glob("resource/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="course",
    maintainer_email="course@university.edu",
    description="TEKS4430 — Assignment 1: Sensing and Perception",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # PROVIDED nodes
            "motion_controller = sensing_assignment.motion_controller:main",
            "ground_truth      = sensing_assignment.ground_truth:main",
            "noise_injector    = sensing_assignment.noise_injector:main",
            # Student task nodes
            "task1_observer    = sensing_assignment.task1_observer:main",
            "task2_filter      = sensing_assignment.task2_filter:main",
            "task3_fusion      = sensing_assignment.task3_fusion:main",
        ],
    },
)
