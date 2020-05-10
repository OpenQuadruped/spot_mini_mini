## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
## Copies from http://docs.ros.org/melodic/api/catkin/html/howto/format2/installing_python.html and edited for our package
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['mini_bullet', 'ars_lib'],
    package_dir={'../mini_bullet/src': ''})

setup(**setup_args)
