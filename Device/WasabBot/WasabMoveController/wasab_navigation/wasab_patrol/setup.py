from setuptools import setup
import os
from glob import glob

package_name = "wasab_patrol"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py") + glob("launch/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gjkong",
    maintainer_email="skong097@gmail.com",
    description="보안관 순찰 코디네이터 (웨이포인트 순회 + 양보)",
    license="MIT",
    entry_points={"console_scripts": ["patrol_node = wasab_patrol.patrol_node:main"]},
)
