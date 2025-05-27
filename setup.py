from setuptools import setup, find_packages
from pathlib import Path

# Read the long description from README.md
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

setup(
    name="neurapy_ai_utils",
    version="0.7.0",
    author="Monika Florek-Jasinska",
    author_email="monika.florek-jasinska@neura-robotics.com",
    description="Robot clients for Neura Robotics AI packages",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    package_data={
        "": ["scripts/*"],
    },
    install_requires=[
        "setuptools",
        # ROS 2 Python client and messages
        "rclpy",
        "std_msgs",
        "sensor_msgs",
        "trajectory_msgs",
        "geometry_msgs",
        "tf2_ros",
        "audio_common_msgs",
        "neura_marker_detection_msgs",
        # Neura Robotics Python libraries
        "neurapy",
        "neura_apps",
        "neurapy_ai",
        # CORBA interface
        "omniORB",
    ],
    python_requires=">=3.6",
    entry_points={
        "console_scripts": [
            "maira_kinematics_node = neurapy_ai_utils.maira_kinematics:main",
        ],
    },
)
