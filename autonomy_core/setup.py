from setuptools import setup

package_name = 'autonomy_core'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/autonomy.launch.py', 'launch/med_competition.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team3',
    maintainer_email='team3@example.com',
    description='Autonomous driving core for medical service robot competition',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'charlie_stage_orchestrator = autonomy_core.stage_orchestrator:main',
            'hazard_handler = autonomy_core.hazard_handler:main',
        ],
    },
)
