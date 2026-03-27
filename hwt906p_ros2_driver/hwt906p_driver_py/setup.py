from setuptools import setup

package_name = 'hwt906p_driver_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hwt906p_python.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wzz',
    maintainer_email='wzz@example.com',
    description='Python ROS 2 driver for the WitMotion HWT906P IMU.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hwt906p_node_py = hwt906p_driver_py.hwt906p_node_py:main',
        ],
    },
)
