from setuptools import setup

package_name = 'lab1_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/lab1_launch.py'] )
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Solomon Fenton',
    maintainer_email='sfenton@andrew.cmu.edu',
    description='F1/10th LAB1 PACKAGE',
    license='NA',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = lab1_pkg.talker:main',
            'relay = lab1_pkg.relay:main',
        ],
    },
)
