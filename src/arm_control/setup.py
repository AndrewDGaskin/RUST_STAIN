from setuptools import find_packages, setup

package_name = 'arm_control'

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
    maintainer='andrew-gaskin',
    maintainer_email='drewgaskin45@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'OPfeedback = arm_control.OPfeedback:main',
        'GScontroller_io = arm_control.GScontroller_io:main',
        'GSfeedback_subscriber = arm_control.GSfeedback_subscriber:main',
        'GUI = arm_control.GUI:main',
        ],
    },
)
