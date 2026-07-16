from glob import glob

from setuptools import find_packages, setup

package_name = 'shelfbot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/web', glob('web/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='이기문',
    maintainer_email='deeptree.test1@gmail.com',
    description='자율주행(pinky) + 모방학습(soarm101) 무인 상품 진열 시스템',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 'orchestrator = shelfbot.orchestrator_node:main',
            # 'docking = shelfbot.aruco_docking:main',
        ],
    },
)
