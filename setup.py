from setuptools import setup, find_packages

setup(
    name='sonar_3d_reconstruction',
    version='1.0.0',
    packages=find_packages(),
    package_data={'sonar_3d_reconstruction': ['sonar_3d_reconstruction_cpp*.so']},
)