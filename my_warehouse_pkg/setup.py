import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_warehouse_pkg'

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 👇 여기서부터 우리가 추가한 폴더들을 등록합니다!
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        # (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
        # (os.path.join('share', package_name, 'models'), glob('models/*.xacro')),
    ]

for path, folders, files in os.walk('models'):
    for file in files:
        source_file = os.path.join(path, file)
        install_path = os.path.join('share', package_name, path)
        data_files.append((install_path, [source_file]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taeyoung',
    maintainer_email='taeyoung@todo.todo',
    description='My automated warehouse simulation package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'conveyor_control = my_warehouse_pkg.conveyor_control_node:main',
            'box_detector = my_warehouse_pkg.box_detector:main',
            'vision_pick_node = my_warehouse_pkg.vision_pick_node:main',
            'vision_pick_node_copy = my_warehouse_pkg.vision_pick_node_copy:main',
        ],
    },
)