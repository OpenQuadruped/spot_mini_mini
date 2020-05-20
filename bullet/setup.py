from setuptools import setup, find_packages

setup(
   name='mb',
   version='1.0',
   description='Pybullet Minitaur Environment and ARS Library',
   author='Maurice Rahme',
   author_email='maurierahme20202@u.northwestern.edu',
   package_dir={'':'src'},
   packages=find_packages('src')
)

# sudo python setup.py build
# THEN
# sudo python setup.py install

