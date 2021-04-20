from setuptools import find_packages
from setuptools import setup

package_name = 'ros2env'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.6.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share_dir, ['package.xml']),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Jeremie Deray, Pyo, Darby Lim',
    author_email='jeremie.deray@canonical.org, passionvirus@gmail.com, routiful@gmail.com',
    maintainer='Pyo',
    maintainer_email='passionvirus@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS 2 example package for the ROS 2 command line tools.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'env = ros2env.command.env:EnvCommand',
        ],
        'ros2cli.extension_point': [
            'ros2env.verb = ros2env.verb:VerbExtension',
        ],
        'ros2env.verb': [
            'list = ros2env.verb.list:ListVerb',
            'set = ros2env.verb.set:SetVerb',
        ],
    },
)
