from setuptools import find_packages, setup

package_name = 'audio_bridge'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team3',
    maintainer_email='team3@example.com',
    description='Voice relay bridge for barcode announcements',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_relay = audio_bridge.audio_relay:main',
        ],
    },
)
