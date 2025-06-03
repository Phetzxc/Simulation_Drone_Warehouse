from setuptools import setup

package_name = 'sjtu_drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='georg.novtony@aon.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = sjtu_drone_control.teleop:main',
            'teleop_joystick = sjtu_drone_control.teleop_joystick:main',
            'open_loop_control = sjtu_drone_control.open_loop_control:main',
            'drone_position_control = sjtu_drone_control.drone_position_control:main',
            'rack_navigator = sjtu_drone_control.rack_navigator:main',
            'aruco_reader = sjtu_drone_control.aruco_reader:main',
            'gen_via_point_path = sjtu_drone_control.gen_via_point_path:main',
            'trajectory_gen = sjtu_drone_control.trajectory_gen:main',
            'aruco_full_pipeline = sjtu_drone_control.aruco_full_pipeline:main',
            'drone_control = sjtu_drone_control.drone_control:main',
            'drone_auto_mode = sjtu_drone_control.drone_auto_mode:main',
            'land_drone_after_finish = sjtu_drone_control.land_drone_after_finish:main',

        ],
    },
)
