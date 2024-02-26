import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'how_is_it_to_be_human_interaction'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oliver-aberle',
    maintainer_email='Oliver.Aberle@student.kit.edu',
    description='Package taking care of the main interaction of my thesis',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'how_is_it_to_be_human_interaction_node = how_is_it_to_be_human_interaction.how_is_it_to_be_human_interaction_node:main' 
        ],
    },
)
