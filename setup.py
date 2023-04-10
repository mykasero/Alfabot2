import os
from glob import glob
from setuptools import setup

package_name = 'alphabot2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alphabot2',
    maintainer_email='alphabot2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_driver = alphabot2.motion_driver:main',
            'IR_obstacle_sensors = alphabot2.IR_obstacle_sensors:main',
            'virtual_odometer = alphabot2.virtual_odometer:main',
            'QR_detector = alphabot2.QR_detector:main',
            'testnode = alphabot2.testnode:main',
            'Avoiding = alphabot2.Avoiding:main',
            'QRmovement = alphabot2.QRmovement:main',
            'path_drawer = alphabot2.path_drawer:main'
        ],
    },
)
