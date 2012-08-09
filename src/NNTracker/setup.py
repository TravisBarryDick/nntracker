from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

import numpy as np

ext_modules = [
    Extension("utility", ["utility.pyx"]),
    Extension("ESMTracker", ["ESMTracker.pyx"]),
    Extension("BMICTracker", ["BMICTracker.pyx"]),
    Extension("NNTracker", ["NNTracker.pyx"])
]

setup(
    name = "NNTracker Library",
    cmdclass = {'build_ext': build_ext},
    include_dirs = [np.get_include()],
    ext_modules = ext_modules
)
    
