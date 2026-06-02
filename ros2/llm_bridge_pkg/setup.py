from setuptools import find_packages, setup

package_name = 'llm_bridge_pkg'

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
    maintainer='tyqwd',
    maintainer_email='tyqwd@todo.todo',
    description='ROS2 bridge node for posting sensor data to the LLM report backend.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'llm_bridge = llm_bridge_pkg.llm_bridge_node:main',
        ],
    }
)
