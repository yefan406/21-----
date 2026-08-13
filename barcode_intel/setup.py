from setuptools import find_packages, setup

package_name = 'barcode_intel'

setup(
    name=package_name,
    version='1.0.0',
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
    description='Barcode intelligence module',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'barcode_reader = barcode_intel.barcode_reader:main'
        ],
    },
)
