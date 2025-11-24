from setuptools import setup
import os
from glob import glob

package_name = 'slambot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**/*', recursive=True)),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/**/*', recursive=True)),
        (os.path.join('share', package_name, 'config'), glob('config/**/*', recursive=True)),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Karol Głąbik',
    maintainer_email='kargla@tlen.pl',
    description='The slambot description package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
