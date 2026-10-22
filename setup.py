from glob import glob

from setuptools import find_packages, setup

package_name = 'pointcloud_filters'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    extras_require={'mesh': ['open3d>=0.14']},
    zip_safe=True,
    maintainer='common',
    maintainer_email='kasahara.yuichiro.res@gmail.com',
    description='Crop around the backhoe position and publish PointCloud2 or Mesh.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_filter = pointcloud_filters.pointcloud_filter_node:main',
        ],
    },
)
