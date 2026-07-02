from setuptools import find_packages, setup

package_name = 'sky_vision2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/zed_mavros_sitl.launch.py',
            'launch/zed_mavros_fc.launch.py',
            'launch/mavros_fc.launch.py',
            'launch/zed.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/apm_pluginlists_vision.yaml',
            'config/fastdds_no_shm.xml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sky',
    maintainer_email='sky@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = sky_vision2.main:main',
            'zed_mavros_bridge = sky_vision2.zed_mavros_bridge:main',
            'test_zed_odom = sky_vision2.test_zed_odom:main',
            'ekf_home_watchdog = sky_vision2.ekf_home_watchdog:main',
        ],
    },
)
