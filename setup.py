from setuptools import setup, Extension
import glob
import os

# Use glob to find the actual .so file with cp310-... in its name
so_file = glob.glob(os.path.join('py_dismech',
                                 'py_dismech*.so'))[0]  # Adjust path as needed

setup(
    name='py_dismech',
    version='0.1',
    packages=['py_dismech'],
    package_dir={'py_dismech': 'py_dismech'},
    package_data={'py_dismech':
                  [os.path.basename(so_file)]},  # Include the .so file
)
