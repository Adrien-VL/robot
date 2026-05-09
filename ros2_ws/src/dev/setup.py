import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dev'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),

        # Launch config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),

        # Rviz config files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),

        # URDF description files
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro', recursive=True)),
        (os.path.join('share', package_name, 'description/robot'), glob('description/robot/*.xacro', recursive=True)),
        (os.path.join('share', package_name, 'description/macros'), glob('description/macros/*.xacro', recursive=True)),
        (os.path.join('share', package_name, 'description/properties'), glob('description/properties/*.xacro', recursive=True)),

        # SDF world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),

        # Map world files
        (os.path.join('share', package_name, 'map'), glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nihl',
    maintainer_email='adrien.vanlancker@ugent.be',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sync_relay = dev.sync:main',
        ],
    },
)
