from setuptools import find_packages, setup
from template.render_jinja_py import render_jinja
from pathlib import Path
import os

package_name = 'dashboard_interface_ros2_node'

base_path_str = str(Path(str(Path(__file__).resolve().parent) + '/'))
template_path = Path(base_path_str + '/' + package_name + '/generated')
template_path.mkdir(exist_ok=True)
os.close(os.open(str(template_path) + '/__init__.py', os.O_CREAT))

render_jinja(base_path_str + '/' + 'template/parameters.py.j2', base_path_str + '/' + package_name+ '/generated/parameters.py')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'template']),
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rhilton',
    maintainer_email='robert.a.hilton.jr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_interface_ros2_node = dashboard_interface_ros2_node.dashboard_interface_ros2_node:main',
        ],
    },
)
