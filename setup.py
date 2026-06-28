from setuptools import setup, find_packages

package_name = 'interceptor_project'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='divyansh',
    description='High-reliability vision-guided tailsitter interceptor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # The main flight control node
            'truth_node = interceptor_ros.truth_node:main',
            # The new 3D visualization node (Updated to perfectly match the filename)
            'visualization_node = interceptor_ros.visualization_node:main'
        ],
    },
)
