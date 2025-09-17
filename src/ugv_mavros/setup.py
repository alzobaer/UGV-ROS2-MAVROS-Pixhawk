from setuptools import setup
package_name = 'ugv_mavros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ugv_mavros']),
        ('share/' + package_name + '/launch', ['launch/ugv_mavros.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zobaer',
    maintainer_email='you@example.com',
    description='UGV wrapper for MAVROS (ROS 2)',
    license='MIT',
    entry_points={'console_scripts': []},
)