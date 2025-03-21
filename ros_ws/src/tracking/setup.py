from setuptools import setup

package_name = 'tracking'

setup(
    name=package_name,
    version='0.2.0',
    packages=['motpy'],
    package_dir={'': 'src'},
    maintainer='Thomas Petrie',
    maintainer_email='thomas.petrie@polymtl.ca',
    description='The tracking package',
    license='MIT',
    install_requires=['setuptools'],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [ 'tracking_node = track:main'],
    },
)