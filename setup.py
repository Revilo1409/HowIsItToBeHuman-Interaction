from setuptools import find_packages, setup

package_name = 'sarai_chatgpt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oliver-aberle',
    maintainer_email='oliver-aberle@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpt_requester_node = sarai_chatgpt.gpt_requester_node:main',
        ],
    },
)
