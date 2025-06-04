from setuptools import setup, find_packages

package_name = 'vehicle_mocap_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['vehicle_mocap_odom', 'vehicle_mocap_odom.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyquaternion',
    ],
    zip_safe=True,
    maintainer='e-krantz',
    maintainer_email='eliaskra@kth.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'vehicle_mocap_odom_node = vehicle_mocap_odom.vehicle_mocap_odom:main',
        ],
    },
)
