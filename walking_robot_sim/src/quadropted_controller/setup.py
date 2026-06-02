#!/usr/bin/env python3
"""Setup.py для регистрации entry points."""
from setuptools import setup, find_packages

setup(
    name='quadropted_controller',
    version='0.1.0',
    packages=find_packages(),
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='RedAlexDad',
    maintainer_email='boss6852@gmail.com',
    description='Quadruped robot controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dog_odometry = scripts.QuadrupedOdometryNode:main',
        ],
    },
)
