from setuptools import find_packages, setup

package_name = 'ngc_propulsion_sim'

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
            "ngc_propulsion_sim = ngc_propulsion_sim.ngc_propulsion_sim:main"
        ],
    },
)
