from setuptools import setup

package_name = 'pixelbot_motors'

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
    maintainer='maure',
    maintainer_email='romain.maure@epfl.ch',
    description='Motors control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pixelbot_motors_node = pixelbot_motors.pixelbot_motors_node:main'
        ],
    },
)
