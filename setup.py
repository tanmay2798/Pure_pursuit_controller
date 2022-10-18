import os
from setuptools import setup
from glob import glob


def get_version(filename):
    import ast

    version = None
    with open(filename) as f:
        for line in f:
            if line.startswith("__version__"):
                version = ast.parse(line).body[0].value.s
                break
        else:
            raise ValueError("No version found in %r." % filename)
    if version is None:
        raise ValueError(filename)
    return version


install_requires = [
    'setuptools',
]
module = "pure_pursuit_controller"
package = "pure_pursuit_controller"
src = "src"

version = get_version(filename=f"src/{module}/__init__.py")
setup(
    name=package,
    version=version,
    # Packages to export
    package_dir={"": src},
    packages=[module],
    # Files we want to install, specifically launch files
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package]),
        ('share/' + package, ['package.xml']),
        (os.path.join('share', package, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package, 'param'), glob('param/*.yaml')),
    ],
    # This is important as well
    install_requires=install_requires,
    zip_safe=True,
    author='Tanmay Goyal',
    author_email='tgoyal@student.ethz.ch',
    maintainer='Tanmay Goyal',
    maintainer_email='tgoyal@student.ethz.ch',
    description='TODO.',
    license='TODO',

    entry_points={
        'console_scripts': [
            "pure_pursuit_node = " + package + ".pure_pursuit_node:main",
            "pure_pursuit_reverse_node = " + package + ".pure_pursuit_reverse_node:main",
            "publish_path = " + package + ".publish_path",
            "publish_start_cmd = " + package + ".publish_start_cmd"
        ],
    },
)
