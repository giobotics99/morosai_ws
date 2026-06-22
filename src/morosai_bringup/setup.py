from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'morosai_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giovanna',
    maintainer_email='giovanna@todo.todo',
    description='MOROSAI AGV bringup package - full system launch',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)
