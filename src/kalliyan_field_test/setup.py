from setuptools import find_packages, setup

package_name = 'kalliyan_field_test'
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
            'seatrac_logger = seatrac_logger.seatrac_logger:main',
            'seatrac_pinger = seatrac_logger.seatrac_pinger:main'
        ],
    },
)
