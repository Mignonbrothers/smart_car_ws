import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'smart_car_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), glob('pc/*.pt') + glob(os.path.join(package_name, '*.pt'))),
        (os.path.join('share', package_name, 'gui', 'maps'), glob(os.path.join(package_name, 'gui', 'maps', '*'))),
        (os.path.join('share', package_name, 'product_images'), glob('product_images/*.png')),
        (os.path.join('share', package_name), ['requirements.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lee',
    maintainer_email='praseodymium99@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pan_tilt_ros2 = smart_car_py_pkg.pan_tilt_ros2:main',
            'servo_udp_bridge = smart_car_py_pkg.servo_udp_bridge:main',
            'cart_gui = smart_car_py_pkg.cart_gui:main',
            'ros2_cart_bridge = smart_car_py_pkg.ros2_cart_bridge:main',
            'robot_gui = smart_car_py_pkg.gui.robot_gui:main',
            'gui_node = smart_car_py_pkg.gui.gui_node:main',
        ],
    },
)
