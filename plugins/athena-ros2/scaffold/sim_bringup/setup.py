from glob import glob

from setuptools import setup

package_name = "sim_bringup"

setup(
    name=package_name,
    version="0.3.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/sim_bringup"]),
        ("share/sim_bringup", ["package.xml"]),
        ("share/sim_bringup/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "startup = sim_bringup.startup:main",
        "contact_monitor = sim_bringup.contact_monitor:main",
    ]},
)
