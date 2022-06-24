from setuptools import setup

package_name = 'path_projection'

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
    maintainer='mpcmeulensteen',
    maintainer_email='mpcmeulensteen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_projection_node = path_projection.path_projection_node:main',
            'multi_path_projection_node = path_projection.multi_path_projection_node:main',
        ],
    },
)
