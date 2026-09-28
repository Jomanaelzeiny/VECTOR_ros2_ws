from setuptools import find_packages, setup

package_name = 'vector_twin_tools'

setup(
    name=package_name,
    version='0.0.3',                                                        # <<< CHANGED
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jomana',
    maintainer_email='jomana@todo.todo',
    description='VECTOR digital twin ROS2 tools: fake camera publisher, twin pose listener, manual control, fake car serial',  # <<< CHANGED
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_camera_publisher = vector_twin_tools.fake_camera_publisher:main',
            'twin_pose_listener = vector_twin_tools.twin_pose_listener:main',
            'keyboard_teleop = vector_twin_tools.keyboard_teleop:main',
            'fake_car_serial = vector_twin_tools.fake_car_serial:main',           # <<< CHANGED
        ],
    },
)