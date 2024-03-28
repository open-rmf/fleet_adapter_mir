from setuptools import find_packages, setup

package_name = 'fleet_adapter_mir_tasks'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Xiyu',
    maintainer_email='xiyu@openrobotics.org',
    description='RMF fleet adapter custom tasks for MiR robots',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'dispatch_delivery = fleet_adapter_mir_tasks.dispatch_delivery:main'
        ],
    },
)
