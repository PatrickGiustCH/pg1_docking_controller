from setuptools import setup

package_name = 'pg1_docking_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pgiust',
    maintainer_email='patrickgiust@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'docking_controller = pg1_docking_controller.docking_controller:main',
                'teleop_node = pg1_docking_controller.teleop_node:main'
        ],
    },
)
