from setuptools import find_packages, setup

package_name = 'object_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hild',
    maintainer_email='poorna.sesetti03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector_node = object_detector.coordinateTransmitter:main',
            'object_detector_with_logs = object_detector.detection_with_logs:main',
            'manual_control = object_detector.manual_control:main',
            'gimbal_pitch_control = object_detector.gimbal_align:main',
            'gimbal_coord = object_detector.gimbal_coords:main'
        ],
    },
)
