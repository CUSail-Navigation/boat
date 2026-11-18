from setuptools import find_packages, setup

package_name = "sailing"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ericcai32",
    maintainer_email="ericcai32@gmail.com",
    description="Sailing control and hardware interfaces.",
    license="Apache License 2.0",
    entry_points={"console_scripts": ["teensy_node = sailing.teensy.teensy_node:main"]},
)
