from setuptools import setup

package_name = 'fleet_adapter_mir'

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
    maintainer='Grey, Aaron',
    maintainer_email='grey@openrobotics.org, aaron@openrobotics.org',
    description='RMF fleet adapter for MiR robots',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'fleet_adapter_mir=fleet_adapter_mir.fleet_adapter_mir:main',
        ],
    },
)
