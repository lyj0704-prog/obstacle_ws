import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch', 'simulation'),
            glob(os.path.join('launch', 'simulation', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ForzaETH',
    maintainer_email='nicolas.baumann@pbl.ee.ethz.ch',
    description='The perception package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracking = perception.tracking1:main',
            'detect = perception.detect1:main',
            'fake_perception_test = '
            'perception.simulation.fake_perception_test:main',
            'perception_result_checker = '
            'perception.simulation.perception_result_checker:main',
            'static_path_detour = perception.simulation.static_path_detour:main',
            'obs_monitor = perception.tools.obs_monitor:main',
        ],
    },
)
