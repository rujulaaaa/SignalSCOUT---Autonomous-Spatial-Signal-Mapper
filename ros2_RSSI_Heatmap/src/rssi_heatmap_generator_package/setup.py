from setuptools import setup

package_name = 'rssi_heatmap_generator_package'
submodules = 'rssi_heatmap_generator_package/submodules'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Collects RSSI data and generates WiFi heatmap overlays',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = rssi_heatmap_generator_package.node:main'
        ],
    },
)
