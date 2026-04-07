from setuptools import find_packages, setup

package_name = 'pb_ur5_sim'

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
            'my_node = pb_ur5_sim.my_node:main',
            'my_node_with_human = pb_ur5_sim.my_node_with_human:main',
            'my_node_robot = pb_ur5_sim.my_node_robot:main',
            'my_node_ursim_display = pb_ur5_sim.my_node_ursim_display:main',
            'my_subscriber = pb_ur5_sim.demo_subscryber:main'
        ],
    },
)
