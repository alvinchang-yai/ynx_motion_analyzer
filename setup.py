from glob import glob

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
        ('share/' + package_name + '/profiles', glob('profiles/*.yaml')),
        ('share/' + package_name + '/dashboard', glob('dashboard/*.*')),
        ('share/' + package_name + '/dashboard/vendor', glob('dashboard/vendor/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='changal',
    maintainer_email='alvin71134@gmail.com',
    description='Record commanded vs. feedback joint motion and analyze the delay in a local dashboard.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'record_motion = motion_trace.record_motion:main',
            'latency_test_example = latency_test_example:main',
            'dashboard = motion_trace.dashboard:main',
        ],
    },
)
