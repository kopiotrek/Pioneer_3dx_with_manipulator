import os
from glob import glob
from setuptools import setup
from setuptools import find_packages
package_name = 'ariadna_simulation_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models'), glob('models/walk.dae')),

        (os.path.join('share', package_name, 'models/cafe_table'), glob('models/cafe_table/model.sdf')),
        (os.path.join('share', package_name, 'models/cafe_table'), glob('models/cafe_table/model.config')),
        (os.path.join('share', package_name, 'models/cafe_table/materials/textures'), glob('models/cafe_table/materials/textures/*.jpg')),
        (os.path.join('share', package_name, 'models/cafe_table/meshes'), glob('models/cafe_table/meshes/*.dae')),

        (os.path.join('share', package_name, 'models/ground_plane'), glob('models/ground_plane/model.sdf')),
        (os.path.join('share', package_name, 'models/ground_plane'), glob('models/ground_plane/model.config')),

        (os.path.join('share', package_name, 'models/sun'), glob('models/sun/model.sdf')),
        (os.path.join('share', package_name, 'models/sun'), glob('models/sun/model.config')),
        (os.path.join('share', package_name, 'models/sun/materials/textures'), glob('models/sun/materials/textures/*.jpg')),
        (os.path.join('share', package_name, 'models/sun/meshes'), glob('models/sun/meshes/*.dae')),

        (os.path.join('share', package_name, 'models/willowgarage'), glob('models/willowgarage/model.sdf')),
        (os.path.join('share', package_name, 'models/willowgarage'), glob('models/willowgarage/model.config')),
        (os.path.join('share', package_name, 'models/willowgarage/materials/textures'), glob('models/willowgarage/materials/textures/*.jpg')),
        (os.path.join('share', package_name, 'models/willowgarage/meshes'), glob('models/willowgarage/meshes/*.dae')),

        (os.path.join('share', package_name, 'models/big_box'), glob('models/big_box/*.*')),
        (os.path.join('share', package_name, 'models/big_box/meshes'), glob('models/big_box/meshes/*')),
        (os.path.join('share', package_name, 'models/big_box/materials/textures'), glob('models/big_box/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wiktor',
    maintainer_email='wiktorbajor1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
