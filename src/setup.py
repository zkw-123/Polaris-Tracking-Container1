from setuptools import setup, find_packages

package_name = 'ros2_ndi_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 wrapper for NDI tracker',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ndi_tracker_node = ros2_ndi_tracker.ndi_tracker_node:main',
            'client_node = ros2_ndi_tracker.client_node:main',
        ],
    },
)