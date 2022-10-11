from setuptools import setup

package_name = 'my_py_pkg'

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
    maintainer='sp',
    maintainer_email='pattarsuraj@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # py_node should not be named same as the name of the file or the name of the node
            "py_node = my_py_pkg.my_first_node:main",
            # Using the same name as the file and the node
            "robot_news_station = my_py_pkg.robot_news_station:main",
            # Smartphone (subscriber) node
            "smartphone = my_py_pkg.smartphone:main",
            # Number publisher node
            "number_publisher = my_py_pkg.number_publisher:main",
            # Number Counter (Subscriber & Publisher) node
            "number_counter = my_py_pkg.number_counter:main",
            # two ints server
            "add_two_ints_server = my_py_pkg.add_two_ints_server:main",
            # two ints client
            "add_two_ints_client_no_oop = my_py_pkg.add_two_ints_client_no_oop:main",
            # two ints client with oop
            "add_two_ints_client = my_py_pkg.add_two_ints_client:main",
            # Hardware status publisher
            "hw_status_publisher = my_py_pkg.hw_status_publisher:main",
            # Led panel
            "led_panel = my_py_pkg.led_panel:main",
            # Battery
            "battery = my_py_pkg.battery:main",
        ],
    },
)
