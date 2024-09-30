from setuptools import find_packages, setup

package_name = 'map_explorer'
submodules = "map_explorer/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(submodules, exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brog',
    maintainer_email='s4750003@students.uq.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_explorer = map_explorer.map_explorer:main'
        ],
    },
)
