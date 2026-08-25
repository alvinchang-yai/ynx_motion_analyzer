from setuptools import find_packages, setup

package_name = 'ynx_motion_analyzer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='changal',
    maintainer_email='alvin71134@gmail.com',
    description='Record and plot commanded vs. feedback joint motion from ynx_hardware_interface.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'record_motion = ynx_motion_analyzer.record_motion:main',
            'plot_motion = ynx_motion_analyzer.plot_motion:main',
            'latency_test_example = ynx_motion_analyzer.latency_test_example:main',
        ],
    },
)
