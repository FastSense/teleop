import os
from glob import glob
from setuptools import setup

package_name = 'teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'pygame'
        ],
    zip_safe=True,
    maintainer='KostyaYamshanov',
    maintainer_email='k.yamshanov@fastsense.tech',
    description='',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'keyboard_listener = teleop.keyboard_listener:main',
        'rosbot_teleop = teleop.rosbot_teleop:main', 
        ],
    },
)
