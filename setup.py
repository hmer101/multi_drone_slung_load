import os
from setuptools import setup
from glob import glob

package_name = 'multi_drone_slung_load'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/utils.py']),
        ('lib/' + package_name, glob(package_name + '/frame_transforms/build/*.so')),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harvey',
    maintainer_email='hmer101@mit.edu',
    description='Control multiple PX4 drones for collaborative load carrying',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone = multi_drone_slung_load.drone:main',
            'gcs_background = multi_drone_slung_load.gcs_background:main',
            'gcs_user = multi_drone_slung_load.gcs_user:main',
            'load = multi_drone_slung_load.load:main',
            'logging_test = multi_drone_slung_load.logging_test:main',
        ],
    },
)
