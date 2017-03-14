import versioneer
from setuptools import (setup, find_packages)


setup(name= 'detrot',
      version=versioneer.get_version(),
      cmdclass=versioneer.get_cmdclass(),
      author='SLAC National Accelerator Laboratory',

      packages=find_packages(),
      description='Python framework for manipulating the CXI detector stands',
      long_description=with open('README.rst') as f: f.read(),
      classifiers=[
          'Programming Language :: Python :: 3.4',
          'Topic :: Robotics, EPICS'
      ],
    )
