from setuptools import find_packages, setup
import glob

package_name = 'actions_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/launch_servers.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eliseo',
    maintainer_email='eliseonunezvega@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'find_ball_server = actions_py.find_ball_server:main',
            'go_to_ball_server = actions_py.go_to_ball_server:main',
            'go_to_ball_pose_server = actions_py.go_to_ball_pose_server:main',
            'go_to_ball_client = actions_py.go_to_ball_client:main'
        ],
    },
)
