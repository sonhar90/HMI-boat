from setuptools import find_packages, setup

package_name = 'kontrollsystemSondre'

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
    maintainer='sherlock',
    maintainer_email='sondreharam@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kontrollerSon = kontrollsystemSondre.kontrollerSon:main',  # For kontroller node
            'estimator_node = kontrollsystemSondre.estimator_node:main',  # For estimator node
        ],
    },
)

