from setuptools import find_packages, setup

package_name = 'patrol_global_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'],
    scripts=[],
    entry_points={'console_scripts': [
        'keyframe_saver = patrol_global_localization.keyframe_saver:main',
        'localization_manager = patrol_global_localization.localization_manager:main',
        'build_map_db = patrol_global_localization.build_map_db:main',
    ]},
)
