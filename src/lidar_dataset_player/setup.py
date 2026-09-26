from setuptools import setup
p='lidar_dataset_player'; setup(name=p,version='0.2.0',packages=[p],data_files=[('share/ament_index/resource_index/packages',['resource/'+p]),('share/'+p,['package.xml'])],install_requires=['setuptools'],entry_points={'console_scripts':['player=lidar_dataset_player.player:main']})
