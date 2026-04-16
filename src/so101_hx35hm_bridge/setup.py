from setuptools import setup

package_name = "so101_hx35hm_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/so101_hx35hm_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="so101_user",
    maintainer_email="you@example.com",
    description="Bridge node from so101 ros2_control forward controller to HX-35HM bus servos via ros_robot_controller SDK.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hx35hm_bridge = so101_hx35hm_bridge.bridge_node:main",
            "aruco_detector = so101_hx35hm_bridge.aruco_detector_node:main",
            "red_circle_detector = so101_hx35hm_bridge.red_circle_detector_node:main",
        ],
    },
)
