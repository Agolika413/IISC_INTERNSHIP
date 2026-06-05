from setuptools import setup
import os
from glob import glob

package_name = 'asmc_ftc_octorotor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models', 'octorotor'), glob('models/octorotor/*')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='ASMC Fault-Tolerant Control for Octorotor in Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'asmc_node = asmc_ftc_octorotor.asmc_node:main',
            'ca_node = asmc_ftc_octorotor.ca_node:main',
            'fault_injector = asmc_ftc_octorotor.fault_injector_node:main',
        ],
    },
)
