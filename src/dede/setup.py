from setuptools import setup
from glob import glob
import os

package_name = 'dede'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.model')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config/slam'), glob('config/slam/*.yaml')),
        (os.path.join('share', package_name, 'config/nav2'), glob('config/nav2/*.yaml')),

        # Model files
        (os.path.join('share', package_name, 'models/dede'), glob('models/dede/*.config')),
        (os.path.join('share', package_name, 'models/dede'), glob('models/dede/*.sdf')),
        (os.path.join('share', package_name, 'models/dede/meshes'), glob('models/dede/meshes/*.stl')),
        (os.path.join('share', package_name, 'models/dede_maze'), glob('models/dede_maze/*.config')),
        (os.path.join('share', package_name, 'models/dede_maze'), glob('models/dede_maze/*.sdf')),
        (os.path.join('share', package_name, 'models/dede_maze/meshes'), glob('models/dede_maze/meshes/*.stl'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marian',
    maintainer_email='mc.lepadatu@noxrobotics.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
