from setuptools import setup
package_name = "wasab_robot_agent"
setup(
    name=package_name, version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="gjkong", maintainer_email="skong097@gmail.com",
    description="WaSaB robot agent", license="MIT",
    entry_points={"console_scripts": [f"agent = {package_name}.agent_node:main"]},
)
