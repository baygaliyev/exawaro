from setuptools import setup, find_packages

setup(
    name='exawaro',
    version='0.1',
    packages=find_packages(),
    install_requires=[
        'osmnx', 'networkx', 'pandas', 'matplotlib', 'numpy', 'tqdm', 'pyproj'
    ],
    author='Gurban Aliyev',
    description='Simulation for exposure-aware pedestrian and car routing'
)
