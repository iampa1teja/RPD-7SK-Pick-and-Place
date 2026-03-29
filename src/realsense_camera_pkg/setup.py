from setuptools import find_packages, setup

package_name = 'realsense_camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', [
            'models/d435i.sdf',
            'models/red_cube.sdf',
            'models/blue_cube.sdf',
            'models/red_box.sdf',
            'models/blue_box.sdf',
            'models/Box.stl',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pavan',
    maintainer_email='pavan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "spawn_cam   = realsense_camera_pkg.spawn:main",
            "start_cam   = realsense_camera_pkg.camera_sim:main",
            "spawn_cubes = realsense_camera_pkg.cube_spawner:main",
            "cam         = realsense_camera_pkg.cam:main" 
        ],
    },
)
