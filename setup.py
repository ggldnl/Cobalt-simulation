from setuptools import find_packages, setup
from glob import glob
import os  


package_name = 'cobalt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'description', 'meshes'), glob('description/meshes/*')),
        (os.path.join('share', package_name, 'description', 'urdf'), glob('description/urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        # (os.path.join('share', package_name, 'meshes'), glob('meshes/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='danielgigliotti99.dg@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
