import os
from setuptools import setup
from glob import glob

package_name = 'meldog_bridge_old'

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
            'listener = meldog_bridge_old.listener:main',
            'multi_moteus_controller = meldog_bridge_old.multi_moteus_controller:main',
            'leg_inv_kin_solver = meldog_bridge_old.leg_inv_kin:main',
            'circle_trajectory = meldog_bridge_old.circle_trajectory:main',
            'demo_single_motor = meldog_bridge_old.demo_single_motor:main',
            'ploter_demo = meldog_bridge_old.ploter:main',
            'multi_moteus_controller_new = meldog_bridge_old.multi_moteus_controller_new:main',
            'linear_trajectory = meldog_bridge_old.linear_trajectory:main',
            'jump_control_node = meldog_bridge_old.jump_control_node:main',
        ],
    },
)
