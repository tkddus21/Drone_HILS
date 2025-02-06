from setuptools import find_packages, setup

package_name = 'lidar_distance'

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
    maintainer='krristudent',
    maintainer_email='krristudent@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'left_lidar = lidar_distance.left_lidar_distance:main',
            'right_lidar = lidar_distance.right_lidar_distance:main',
        ],
    },
)
