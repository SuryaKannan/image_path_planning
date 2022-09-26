from setuptools import setup

package_name = 'waypoint_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suryak',
    maintainer_email='skan0017@student.monash.edu',
    description='publish global waypoints',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'waypoint_publisher = waypoint_generator.waypoint_publisher_node:main'
        ],
    },
)
