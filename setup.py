from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mini_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bf01',
    maintainer_email='fanbenlong@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_square = mini_project.drive_square:main',
            'teleop = mini_project.teleop:main',
            'wall_follower = mini_project.wall_follower:main',
            'wall_detector = mini_project.wall_detector:main',
            'person_follower = mini_project.person_follower:main'
        ],
    },
)
