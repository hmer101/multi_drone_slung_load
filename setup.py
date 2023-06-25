import os
from setuptools import setup
from glob import glob

package_name = 'swarm_load_carry'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/utils.py']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        #(os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
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
            'drone = swarm_load_carry.drone:main',
            'gcs_background = swarm_load_carry.gcs_background:main',
            'gcs_user = swarm_load_carry.gcs_user:main',
            'load = swarm_load_carry.load:main',
            #'offboard_test = swarm_load_carry.offboard_test:main',
        ],
    },
)
