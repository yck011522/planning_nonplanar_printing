from setuptools import setup, find_packages

setup(
    name="planning_nonplanar_printing",  # Replace with your project name
    version="0.1.0",
    description="A temporary code base for upstreaming ITJ and PyChoero code ",
    
    # Automatically find the packages in current directory
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    
    # Dependencies
    install_requires=[
        'compas',
        'compas_fab',
        'compas_fab_pychoreo',
        'scipy==1.11.1'
    ],
    # Local packages
    dependency_links=[
        '.\external\compas',
        '.\external\compas_fab',
        '.\external\compas_fab_pychoreo'
    ],
    author="Victor Leung",
)
