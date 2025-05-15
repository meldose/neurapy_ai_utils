from setuptools import setup, find_packages
from pathlib import Path

# python dependencies listed here will be automatically installed with the package
long_description = (Path(__file__).parent / "README.md").read_text()
module_name = "neurapy_ai_utils"

setup(
    name=module_name,
    version="0.7.0.dev10",
    author="Monika Florek-Jasinska",
    author_email="monika.florek-jasinska@neura-robotics.com",
    description=("Robot clients for Neura Robotics AI packages"),
    long_description=long_description,
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    package_data={
        "": ["scripts/*"],
    },
    install_requires=[
        "numpy",
    ],
    python_requires=">=3.6",
)
