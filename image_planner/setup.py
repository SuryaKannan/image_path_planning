from setuptools import setup
from glob import glob
import os

package_name = 'image_planner'
submodules = 'image_planner/utils'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suryak',
    maintainer_email='skan0017@student.monash.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tentacle_planner = image_planner.tentacle_planner_node:main'
        ],
    },
)
