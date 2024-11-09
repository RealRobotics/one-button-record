import os
from glob import glob
from setuptools import find_packages, setup

package_name = "one_button_record"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name + "/config"),
            glob("config/*.ymal"),
        ),
        (
            os.path.join("share", package_name + "/launch"),
            glob("launch/*launch.[pxy][yma]*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="andy",
    maintainer_email="a.j.blight@leeds.ac.uk",
    description="Record compressed camera data to file using a single button to enable and disable the recording.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "one_button_record = one_button_record.one_button_record:main",
            "hw_test = one_button_record.hw_test:main",
        ],
    },
)
