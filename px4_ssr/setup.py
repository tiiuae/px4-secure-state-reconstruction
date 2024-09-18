import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'px4_ssr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juniorsundar-unikie',
    maintainer_email='junior.sundar@tii.ae',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_estimator = px4_ssr.state_estimator:main',
            'safe_controller = px4_ssr.safe_controller:main',
        ],
    },
)
