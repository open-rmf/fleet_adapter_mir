from setuptools import setup

package_name = 'fleet_adapter_mirfm'

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
    maintainer='Xi Yu Oh',
    maintainer_email='xiyu@openrobotics.org',
    description='RMF fleet adapter for MiR Fleet Manager',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'fleet_adapter_mirfm=fleet_adapter_mirfm.fleet_adapter_mirfm:main',
        ],
    },
)
