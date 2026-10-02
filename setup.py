from setuptools import setup, find_packages

with open('README.md', encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='exawaro',
    version='0.2.0',
    description='Exposure-aware routing and behavioural simulation on urban road networks',
    long_description=long_description,
    long_description_content_type='text/markdown',
    author='Gurban Aliyev',
    url='https://github.com/baygaliyev/exawaro',
    license='MIT',
    packages=find_packages(exclude=['tests', 'tests.*']),
    python_requires='>=3.9',
    install_requires=[
        'geopandas',
        'networkx',
        'numpy',
        'osmnx',
        'pandas',
        'pyproj',
        'shapely',
    ],
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Topic :: Scientific/Engineering :: GIS',
    ],
)