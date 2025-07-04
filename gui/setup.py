import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*/*.top')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*/*.wrl')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='knight',
    maintainer_email='levacarrillo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    package_data={
        'gui': ['resources/*.png'],
    },
    include_package_data=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator = gui.main:main',
        ],
    },
)
