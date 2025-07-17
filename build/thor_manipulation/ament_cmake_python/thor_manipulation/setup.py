from setuptools import find_packages
from setuptools import setup

setup(
    name='thor_manipulation',
    version='0.1.0',
    packages=find_packages(
        include=('thor_manipulation', 'thor_manipulation.*')),
)
