from setuptools import find_packages, setup
import glob

package_name = 'udp_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch*")),
    ],
    install_requires=['launch', 'setuptools'],
    zip_safe=True,
    maintainer='sebastian',
    maintainer_email='sebastian.stelter@cranfield.ac.uk',
    description='TODO: Package description',
    license='MIT',
    entry_points={
        'console_scripts': [
            f"receiver = {package_name}.receiver:main",
            f"sender = {package_name}.sender:main",
        ],
    },
)
