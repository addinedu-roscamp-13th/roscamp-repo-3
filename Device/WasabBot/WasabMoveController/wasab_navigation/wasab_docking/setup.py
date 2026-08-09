from setuptools import setup
package_name = "wasab_docking"
setup(
    name=package_name, version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/precision_parking.yaml", "config/relocalizer.yaml"]),
        (f"share/{package_name}/launch", ["launch/precision_parking.launch.py", "launch/relocalization.launch.py", "launch/dock.launch.py"]),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="gjkong", maintainer_email="skong097@gmail.com",
    description="WaSaB AprilTag PID 정밀 주차", license="MIT",
    entry_points={"console_scripts": [
        f"apriltag_detector = {package_name}.apriltag_detector_node:main",
        f"precision_parking = {package_name}.precision_parking_node:main",
        f"apriltag_relocalizer = {package_name}.apriltag_relocalizer_node:main",
    ]},
)
