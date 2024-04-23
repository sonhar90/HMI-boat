from setuptools import find_packages, setup

package_name = 'ngc_sensor_sims'

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
    maintainer='oivind',
    maintainer_email='oivind@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
	'console_scripts': [
		'gnss = ngc_sensor_sims.gnss:main',
		'compass = ngc_sensor_sims.compass:main',
		'anemometer = ngc_sensor_sims.anemometer:main',
		],
	},
)
