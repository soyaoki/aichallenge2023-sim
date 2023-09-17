from setuptools import find_packages, setup

package_name = 'forced_turnaround_operator'

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
    maintainer='regulus',
    maintainer_email='-@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forced_turnaround_operator = forced_turnaround_operator.forced_turnaround_operator:main'
        ],
    },
)
