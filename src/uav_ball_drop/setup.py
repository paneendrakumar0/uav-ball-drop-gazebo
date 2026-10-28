import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'uav_ball_drop'

def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    
    # Recursively collect models, worlds, launch, urdf
    for subfolder in ['models', 'worlds', 'launch', 'urdf', 'config']:
        for root, dirs, files in os.walk(subfolder):
            if files:
                install_dir = os.path.join('share', package_name, root)
                file_paths = [os.path.join(root, f) for f in files]
                data_files.append((install_dir, file_paths))
                
    return data_files

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bathala Paneendra Kumar',
    maintainer_email='bpk.24u10048@nitdgp.ac.in',
    description='ROS 2 Jazzy & Gazebo Sim Harmonic package for UAV precision ball drop mechanism',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_drop_controller = uav_ball_drop.ball_drop_controller:main',
            'test_drop_client = uav_ball_drop.test_drop_client:main',
        ],
    },
)
