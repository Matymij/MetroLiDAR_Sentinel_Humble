from setuptools import setup
p='metro_obstacle_detector'; setup(name=p,version='0.2.0',packages=[p],data_files=[('share/ament_index/resource_index/packages',['resource/'+p]),('share/'+p,['package.xml'])],install_requires=['setuptools'],zip_safe=True,entry_points={'console_scripts':['detector_node=metro_obstacle_detector.detector_node:main']})
