from setuptools import find_packages, setup

package_name = 'get_ball_coordinates'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', package_name +'/tennis_ball_classifier.xml']),
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
            'detect = get_ball_coordinates.detect_ball:main',
            'detect3d = get_ball_coordinates.detect_ball3d:main'
        ],
    },
)
