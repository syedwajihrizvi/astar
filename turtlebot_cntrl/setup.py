from setuptools import find_packages, setup

package_name = 'turtlebot_cntrl'

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
    maintainer='syedrizvi',
    maintainer_email='syedrizvi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_node = turtlebot_cntrl.map_node:main',
            'path_planner_node = turtlebot_cntrl.path_planner_node:main',
            'nid_node = turtlebot_cntrl.nid_node:main',
        ],
    },
)
