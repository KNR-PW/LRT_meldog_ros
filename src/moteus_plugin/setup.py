from setuptools import setup
import os
from glob import glob

package_name = 'moteus_plugin'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        (os.path.join('share', package_name), glob('resource/*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jmacuga',
    maintainer_email='julia.macuga@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
