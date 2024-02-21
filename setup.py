import os
from glob import glob
from setuptools import setup

package_name = 'pixelbot_audio'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'sound'), glob('sound/*.mp3')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maure',
    maintainer_email='romain.maure@epfl.ch',
    description='Package managing the audio capabilities of PixelBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pixelbot_audio_node = pixelbot_audio.pixelbot_audio_node:main'
        ],
    },
)
