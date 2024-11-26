from glob import glob
from setuptools import find_packages, setup

package_name = "regulator"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/images",
            glob("resource/images/*"),
        ),  # Include images
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer='vidarkar2002',
    maintainer_email='vidarkar@stud.ntnu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kontroller = regulator.kontroller:main',
            'estimator = regulator.estimator:main',
            'allokering = regulator.allokering:main',
            'guide = regulator.guide:main',
            'hmi = regulator.hmi:main',
            'hmi_gui = regulator.hmi_gui:main',
            'waypoint_controller = regulator.waypoint_controller:main'
        ],
    },
)
