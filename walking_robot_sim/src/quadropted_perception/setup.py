#!/usr/bin/env python3
from setuptools import setup, find_packages

setup(
    name='quadropted_perception',
    version='0.1.0',
    packages=find_packages(),
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='RedAlexDad',
    maintainer_email='boss6852@gmail.com',
    description='YOLO object detection for quadruped robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_detector = quadropted_perception.yolo_detector:main',
            'visualizer = quadropted_perception.visualizer:main',
        ],
    },
)
