from setuptools import setup

package_name = 'ms_agents_pkg'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Simple lidar obstacle avoidance agent',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_agent = ms_agents_pkg.drive_agent:main',
            'simple_lidar_avoider = ms_agents_pkg.simple_lidar_avoider:main',
            'safety_braking_agent = ms_agents_pkg.safety_braking_agent:main',
            'pid_wall_follower = ms_agents_pkg.pid_wall_follower:main',
        ],
    },
)
