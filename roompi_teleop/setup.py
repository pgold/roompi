from setuptools import setup

package_name = 'roompi_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pgold',
    maintainer_email='pgold@users.noreply.github.com',
    description='RoomPi tele-operation.',
    license='Unlicense',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'key_teleop = roompi_teleop.key_teleop:main',
        ],
    },
)
