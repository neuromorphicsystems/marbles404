# Minimal build script for the `scamp` Python extension on Windows.
#
# Usage:
#   python setup.py build_ext --inplace      # produces scamp.<tag>.pyd here
#   python setup.py install                  # installs into site-packages
#
# This file is a stripped clone of ../scamp_python_module/setup.py. It points
# at the C++ sources and prebuilt static lib already in this repo, so no
# duplication is needed. See ../scamp_python_module/BUILD_WINDOWS.md for a
# full walkthrough.

from setuptools import setup, Extension

SCAMP_MODULE_DIR = '../scamp_python_module'
SCAMP_INCLUDE_DIR = '../scamp5d_interface'
SCAMP_LIB_DIR = '../x64/Release'   # use '../Release' for 32-bit Python

scamp_ext = Extension(
    name='scamp',
    sources=[
        f'{SCAMP_MODULE_DIR}/scampmodule.cpp',
        f'{SCAMP_MODULE_DIR}/scampmodule_packet_switch.cpp',
    ],
    include_dirs=[SCAMP_INCLUDE_DIR, SCAMP_MODULE_DIR],
    library_dirs=[SCAMP_LIB_DIR],
    libraries=['scamp5d_interface', 'kernel32', 'user32', 'advapi32'],
    extra_compile_args=['/O2'],
    extra_link_args=['/LTCG:OFF'],
    language='c++',
)

setup(name='scamp', version='1.0', ext_modules=[scamp_ext])
