from setuptools import find_packages, setup
package_name='patrol_bridge'
setup(name=package_name,version='0.1.0',packages=find_packages(),data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml'])],install_requires=['setuptools'],zip_safe=True,maintainer='byl',maintainer_email='byl@localhost',description='GO2 patrol ROS bridge',license='MIT',entry_points={'console_scripts':['bridge = patrol_bridge.bridge:main','nav_motion_bridge = patrol_bridge.nav_motion_bridge:main']})
