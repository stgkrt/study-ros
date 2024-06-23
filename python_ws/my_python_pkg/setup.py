from setuptools import setup

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        "my_python_pkg.hello",
        "my_python_pkg.hello_loop",
        "my_python_pkg.my_publisher",
        "my_python_pkg.my_subscriber",
        "my_python_pkg.my_turtle_controller",
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='taro.stst@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello=my_python_pkg.hello:main',
            'hello_loop=my_python_pkg.hello_loop:main',
            'my_publisher=my_python_pkg.my_publisher:main',
            'my_subscriber=my_python_pkg.my_subscriber:main',
            "my_turtle_controller=my_python_pkg.my_turtle_controller:main"
        ],
    },
)
