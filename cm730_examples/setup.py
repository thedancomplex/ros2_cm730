from setuptools import setup

package_name = 'cm730_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'head_zero'
    ],
    data_files=[],
    install_requires=['setuptools'],
    maintainer='Sander van Dijk',
    maintainer_email='sgvandijk@gmail.com',
    keywords=['ROS'],
    description='Examples using the CM730-controller',
    entry_points={
        'console_scripts': [
            'head_zero = head_zero:main'
        ],
    },
)
