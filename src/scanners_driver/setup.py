from setuptools import find_packages, setup

package_name = 'scanners_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Karol Głąbik',
    maintainer_email='kargla@tlen.pl',
    description='Custom Lidar driver package',
    license='License Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_driver = scanners_driver.lidar_driver:main'
        ],
    },
)
