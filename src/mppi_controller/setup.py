from setuptools import find_packages, setup

package_name = 'mppi_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sobot',
    maintainer_email='sobot@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mppi_controller = mppi_controller.main:main',
            'mppi_no_human = mppi_controller.main_no_human:main',
            'mppi_robot = mppi_controller.main_robot:main',
            'mppi_rh = mppi_controller.main_rh:main'
        ],
    },
)
