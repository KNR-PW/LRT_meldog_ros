import os
from setuptools import setup
from glob import glob

package_name = 'meldog_leg_tests'

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
            'listener = meldog_leg_tests.listener:main',
            'multi_moteus_controller = meldog_leg_tests.multi_moteus_controller:main',
            '2D_inv_kin_solver = meldog_leg_tests.2D_inverse_kinematics:main',
            'circle_trajectory = meldog_leg_tests.circle_trajectory:main',
            'demo_single_motor = meldog_leg_tests.demo_single_motor:main',
            'ploter_demo = meldog_leg_tests.ploter:main',
            'multi_moteus_controller_new = meldog_leg_tests.multi_moteus_controller_new:main',
            'linear_trajectory = meldog_leg_tests.linear_trajectory:main',
            'jump_control_node = meldog_leg_tests.jump_control_node:main',
        ],
    },
)
