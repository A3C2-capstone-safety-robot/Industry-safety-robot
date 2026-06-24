from setuptools import find_packages, setup

package_name = 'thermal_fusion_pkg'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'thermal_visualizer = thermal_fusion_pkg.thermal_visualizer_node:main',
            'image_fusion = thermal_fusion_pkg.image_fusion_node:main',
        ],
    }
)
