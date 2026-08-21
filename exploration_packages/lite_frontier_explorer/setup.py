from setuptools import find_packages, setup

package_name = 'lite_frontier_explorer'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nic Smith',
    maintainer_email='nicholas@nbembedded.com',
    description="Minimal frontier-exploration node: reads nav2's global costmap, picks the nearest frontier, sends it to nav2.",
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lite_frontier_explorer_node = lite_frontier_explorer.frontier_node:main',
        ],
    },
)
