from setuptools import find_packages, setup

package_name = 'simple_pick_and_place_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rinho',
    maintainer_email='hungdangpham.ai@gmail.com',
    description='Block detection perception pipeline for FR3 Pick and Place',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'block_detector = simple_pick_and_place_perception.block_detector:main',
        ],
    },
)