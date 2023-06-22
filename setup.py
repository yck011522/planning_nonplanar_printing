from setuptools import setup, find_packages

setup(
    name="planning_nonplanar_printing",  # Replace with your project name
    version="0.1.0",
    description="A temporary code base for upstreaming ITJ and PyChoero code ",
    packages=find_packages(where = 'src'),  # Automatically find the packages in current directory
    package_dir={'': 'src'},
    install_requires=[],  # If your project has any dependencies, list them here
    author="Victor Leung",
)