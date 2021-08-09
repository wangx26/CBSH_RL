from distutils.core import setup, Extension

from Cython.Build import cythonize

extension = Extension(
    "cbsrl",
    ["cbsrl.pyx"],
    libraries=["../src/common/mapf_map/mapf_map.cpp"]
)

setup(ext_modules=cythonize("cbsrl.pyx"))