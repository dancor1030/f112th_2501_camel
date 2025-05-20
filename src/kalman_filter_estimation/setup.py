from setuptools import find_packages, setup

package_name = 'kalman_filter_estimation'

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
    maintainer='eia',
    maintainer_email='andresfmc223@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kalman_filter_estimator = kalman_filter_estimation.kalman_filter_estimator:main'
        ],
    },
)
