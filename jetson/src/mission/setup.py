from setuptools import find_packages, setup

package_name = "mission"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="ericcai32",
    maintainer_email="ericcai32@gmail.com",
    description="Mission logic for boat: waypoint queue, modes and buoy search.",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "waypoint_service = mission.waypoint_service:main",
            "mode_manager = mission.mode_manager:main",
            "buoy_search = mission.buoy_search_node:main",
        ],
    },
)
