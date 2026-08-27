from setuptools import find_packages, setup

package_name = 'ynx_motion_analyzer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    py_modules=['latency_test_example'],
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
            'record_motion = motion_trace.record_motion:main',
            'plot_motion = motion_trace.plot_motion:main',
            'plot_shift_check = shift_check.plot_shift_check:main',
            'latency_test_example = latency_test_example:main',
        ],
    },
)
