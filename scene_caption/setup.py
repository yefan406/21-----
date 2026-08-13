from setuptools import find_packages, setup

package_name = 'scene_caption'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'volcengine-python-sdk[ark]'],
    zip_safe=True,
    maintainer='team3',
    maintainer_email='team3@example.com',
    description='Scene caption module for medical scenario interpretation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scene_interpreter = scene_caption.scene_interpreter:main',
            'display_terminal = scene_caption.display_terminal:main',
        ],
    },
)
