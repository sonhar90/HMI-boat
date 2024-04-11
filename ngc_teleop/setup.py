from setuptools import find_packages, setup

package_name = 'ngc_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nupix',
    maintainer_email='aksel.t.frafjord@ntnu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "manual_set_point_arbeidsbaat = ngc_teleop.arbeidsbaat_man_sp:main",
        ],
    },
)
