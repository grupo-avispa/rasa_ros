from glob import glob

from setuptools import find_packages, setup

package_name = 'rasa_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='José Galeas Merchán',
    author_email='jgaleas@uma.es',
    maintainer='Alberto Tudela',
    maintainer_email='ajtudela@gmail.com',
    description='ROS 2 package for integrating Natural Language Understanding using Rasa.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['rasa_ros = rasa_ros.rasa_ros:main'],
    },
)
