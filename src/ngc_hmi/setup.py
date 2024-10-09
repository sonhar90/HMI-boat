from setuptools import find_packages, setup

package_name = 'ngc_hmi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['ngc_hmi', 'ngc_hmi.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oivind',
    maintainer_email='oivindkjerstad@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ngc_hmi = ngc_hmi.ngc_hmi_main:main", 
            "ngc_hmi_autopilot = ngc_hmi.ngc_hmi_autopilot:main", 
            "ngc_hmi_yaml_editor = ngc_hmi.ngc_hmi_yaml_editor:main", 
        ],
    },
)
