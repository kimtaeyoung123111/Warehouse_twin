import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_warehouse_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 👇 여기서부터 우리가 추가한 폴더들을 등록합니다!
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taeyoung',
    maintainer_email='taeyoung@todo.todo',
    description='My automated warehouse simulation package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 나중에 파이썬 노드(예: conveyor_controller)를 만들면 여기에 등록합니다.
            'conveyor_control = my_warehouse_pkg.conveyor_control_node:main',
        ],
    },
)