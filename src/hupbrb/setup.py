from setuptools import setup
import os
from glob import glob

package_name = 'hupbrb'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fontys',
    maintainer_email='fontys@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'id_scanner = hupbrb.IdScanner:main',
            'prio_controller = hupbrb.PriorityController:main',
            'collision_detection = hupbrb.CollisionDetection:main',
        ],
    },
)
