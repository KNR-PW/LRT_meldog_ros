import os
from setuptools import setup
from glob import glob

package_name = 'meldog_bridge'

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
    maintainer='jmacuga',
    maintainer_email='julia.macuga@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = meldog_bridge.listener:main',
            'multi_moteus_controller = meldog_bridge.multi_moteus_controller:main',
            'leg_inv_kin_solver = meldog_bridge.leg_inv_kin:main',
            'circle_trajectory = meldog_bridge.circle_trajectory:main',
            'demo_single_motor = meldog_bridge.demo_single_motor:main',
            'ploter_demo = meldog_bridge.ploter:main',
        ],
    },
)
