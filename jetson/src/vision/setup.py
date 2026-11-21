from glob import glob

from setuptools import find_packages, setup

package_name = "vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        # YOLO weights are gitignored; install them when present.
        (f"share/{package_name}", glob(f"{package_name}/*.pt")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="ericcai32",
    maintainer_email="ericcai32@gmail.com",
    description="Buoy detection from the RealSense camera.",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": ["buoy_detection = vision.camera_node:main"],
    },
)
