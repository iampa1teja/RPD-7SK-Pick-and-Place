from setuptools import find_packages, setup
from glob import glob

package_name = 'segmenta'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/weights', glob('weights/*.pth')),
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
            "start = segmenta.node:main",
            "detect = segmenta.transform_sim:main",
            "cam_sim = segmenta.cam_sim:main",
        ],
    },
)
