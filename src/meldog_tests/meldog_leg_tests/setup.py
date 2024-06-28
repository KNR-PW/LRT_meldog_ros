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
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jmacuga',
    maintainer_email='bartlomiejk@vp.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_moteus_controller_two_threads = meldog_leg_tests.multi_moteus_controller_two_threads:main',
            'inverse_kinematics_2D = meldog_leg_tests.inverse_kinematics_2D:main',
            'circle_trajectory = meldog_leg_tests.circle_trajectory:main',
            'demo_single_motor = meldog_leg_tests.demo_single_motor:main',
            'ploter_demo = meldog_leg_tests.ploter:main',
            'multi_moteus_controller_single_thread = meldog_leg_tests.multi_moteus_controller_single_thread:main',
            'linear_trajectory = meldog_leg_tests.linear_trajectory:main',
            'jump_control_node = meldog_leg_tests.jump_control_node:main',
            'forward_kinematics_2D = meldog_leg_tests.forward_kinematics_2D:main'
        ],
    },
)
