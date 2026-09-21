import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'uas_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch_folder'),
            glob('launch_folder/*.launch.py')),
        ('share/uas_sim/config', glob('config/*.json')),
        ('share/uas_sim/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='orin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'wind_manager = uas_sim.wind.cell_manager:main',
            'wind_command = uas_sim.wind.wind_command:main',
            'wind_summary = uas_sim.wind.summarize:main',
            'wind_fit_envelope = uas_sim.wind.fit_envelope:main',

            'offboard_nav = uas_sim.offboard_nav:main',
            'cell_manager = uas_sim.cell_manager:main',
            "footprint_bubble = uas_sim.footprint_bubble:main",
            'corridor_nav = uas_sim.corridor_nav:main',
            "corridor_view = uas_sim.corridor_view:main",
        ],
    },
)