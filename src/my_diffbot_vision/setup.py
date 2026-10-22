from setuptools import find_packages, setup

package_name = 'my_diffbot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/yolo11n.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ragnarhritz',
    maintainer_email='hritzj1100@gmail.com',
    description='YOLO-based camera perception nodes for DiffBot.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector = my_diffbot_vision.Yolo_detection:main',
        ],
    },
)
