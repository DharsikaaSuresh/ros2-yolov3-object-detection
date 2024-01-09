from setuptools import find_packages, setup
from glob import glob

package_name = 'object_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/**/*', recursive=True)),
        ('share/' + package_name + '/launch', glob('launch/**/*', recursive=True)),
        ('share/' + package_name + '/yolov3_data', glob('yolov3_data/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dharsikaa',
    maintainer_email='dharsikaa23@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector_node = object_detector.main:main',
        ],
    },
)
