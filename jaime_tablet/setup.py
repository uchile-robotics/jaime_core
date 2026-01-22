from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jaime_tablet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch','*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'tablet_scripts'), glob('tablet_scripts/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica',
    maintainer_email='robotica@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'file_publisher = jaime_tablet.file_publisher:main',
          'iriun_publisher = jaime_tablet.iriun_publisher:main',
          'media_sender = jaime_tablet.media_sender:main',
        ],
    },
)
