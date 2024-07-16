from setuptools import find_packages, setup

package_name = 'derek_field_test'

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
    maintainer='claytonsmith',
    maintainer_email='claytonsmith3.14@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'derek_logger = derek_field_test.derek_logger:main',
            'derek_pinger = derek_field_test.derek_pinger:main'
        ],
    },
)
