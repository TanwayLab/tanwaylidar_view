from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup_pkg = generate_distutils_setup(
    packages=['tensorpro_interfaces'], 
    package_dir={'': 'src'},
    scripts=['scripts/tensorpro_interfaces']
)
setup(**setup_pkg)
