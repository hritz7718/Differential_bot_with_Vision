from setuptools import find_packages, setup

package_name = 'my_diffbot_navigation'

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
    maintainer='ragnarhritz',
    maintainer_email='hritzj1100@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_monitor = my_diffbot_navigation.imu_monitor:main',
            'obstacle_avoidance = my_diffbot_navigation.obstacle_avoidance:main',
            'target_follower = my_diffbot_navigation.target_follower:main',
        ],
    },
)
