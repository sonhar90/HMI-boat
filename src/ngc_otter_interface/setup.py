from setuptools import find_packages, setup

package_name = 'ngc_otter_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the parsed_data.csv in the package installation
        ('share/' + package_name + '/resource', ['resource/parsed_data.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oivindk',
    maintainer_email='oivindkjerstad@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "otter_interface = ngc_otter_interface.otter_interface_node:main"
        ],
    },
    package_data={
        # Include the CSV file in the package data
        '': ['resource/*.csv'],
    },
)
