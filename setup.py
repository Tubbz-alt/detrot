from setuptools import (setup, find_packages)


setup(name     = 'detrot',
      version  = versioneer.get_version(),
      cmdclass = versioneer.get_cmdclass(),
      author   = 'SLAC National Accelerator Laboratory',

      packages    = find_packages(),
      description = 'Python framework for manipulating the CXI detector stands'

    )
