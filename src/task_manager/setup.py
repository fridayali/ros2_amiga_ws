from setuptools import find_packages, setup

package_name = 'task_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cuma_karaaslan',
    maintainer_email='cumaali.karaaslan51@gmail.com',
    description='Task manager ROS2 node for Amiga robot mission management',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager_node = task_manager.task_manager_node:main',
        ],
    },
)
