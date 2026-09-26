from setuptools import setup
p='metro_benchmark'; setup(name=p,version='0.2.0',packages=[p],data_files=[('share/ament_index/resource_index/packages',['resource/'+p]),('share/'+p,['package.xml'])],install_requires=['setuptools'],entry_points={'console_scripts':['benchmark_node=metro_benchmark.benchmark_node:main']})
