from setuptools import setup

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
    entry_points={
        'console_scripts': [
            'ndi_tracker_node = ros2_ndi_tracker.ndi_tracker_node:main',
        ],
    }
)
