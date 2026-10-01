from setuptools import find_packages, setup

package_name = 'fusion_py'

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
    maintainer='kzb068',
    maintainer_email='kzb068@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # === Trimmed to 6 nodes (2026-10-01) ===
            # Removed: data_publisher, ComparisonVisualizer(v1/v2/v3),
            #          ControlSim, CANTrackReader, WpsFromJustinApp2
            'CANGPSReader = fusion_py.CanGPSReader:main',
            'SerialGPSReader = fusion_py.SerialGPS:main',
            'WpsFromJustinApp = fusion_py.WpsFromJustinApp:main',
            'VehicleCommander = fusion_py.VehicleCommander:main',
            'VehicleCommanderSRP = fusion_py.VehicleCommanderSRP:main',
        ],
    },
)
