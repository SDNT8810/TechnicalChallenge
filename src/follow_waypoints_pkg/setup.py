from setuptools import find_packages, setup

package_name = 'follow_waypoints_pkg'

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
    maintainer='SDNT8810',
    maintainer_email='davood_space@yahoo.com',
    description='A package to follow PoseStamped waypoints using Nav2 followWaypoints service',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_follower = follow_waypoints_pkg.odom_follower:main',
            'gps_follower  = follow_waypoints_pkg.gps_follower:main',
            'go_through_poses = follow_waypoints_pkg.go_through_poses:main',
        ],
    },
)
