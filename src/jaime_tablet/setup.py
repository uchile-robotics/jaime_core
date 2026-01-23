from setuptools import setup

package_name = 'jaime_tablet'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [
            'launch/tablet_launch.py'
        ]),
        ('share/' + package_name + '/imagenes', [
            'imagenes/funciona.gif',
            'imagenes/bumblebee.gif',
            'imagenes/chimuelo1.gif',
            'imagenes/chimuelo2.gif',
            'imagenes/chimuelo3.gif',
            'imagenes/noloent.gif',
            'imagenes/pato.gif',
            'imagenes/pls.gif'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mari',
    description='Tablet media sender',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'media_sender = jaime_tablet.media_sender:main',
            'file_publisher = jaime_tablet.file_publisher:main',
            'android_cam = jaime_tablet.android_cam:main',
        ],
    },
)
