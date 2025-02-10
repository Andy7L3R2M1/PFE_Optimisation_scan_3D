import glob
import os

from setuptools import setup

package_name = "pol_spatial_data_types"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Polytech Montpellier",
    maintainer_email="valentin.racaud-minuzzi@etu.umontpellier.fr",
    description="Definition of types used in pol_spatial_data packages",
    license="GNU General Public License version 3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
