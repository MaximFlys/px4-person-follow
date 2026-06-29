from setuptools import find_packages, setup

package_name = 'flight_control_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['Stools'],
    zip_safe=True,
    maintainer='maxim',
    maintainer_email='maxim@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'start_flight = flight_control_node.flight_node:main'
        ],
    },
)
