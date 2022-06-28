import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'simple_ball_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='conner',
    maintainer_email='robotics@cnu.edu',
    description='A package to detect colored balls',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'simple_ball_detector=simple_ball_detector.simple_ball_detector:main',
                             'cv2_demo_image_proc=simple_ball_detector.cv2_demo_image_proc:main',
                             'fake_ball_detector=simple_ball_detector.fake_ball_detector:main'
        ],
    },
)
