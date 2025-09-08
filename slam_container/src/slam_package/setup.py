from setuptools import find_packages, setup

package_name = 'slam_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_launch.py']),  # Относительный путь
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexey',
    maintainer_email='alexey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
