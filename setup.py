from setuptools import setup, find_packages

setup(
    name="meshflow",
    version="1.0.0",
    packages=find_packages(exclude=["tests*"]),
    entry_points={
        "console_scripts": [
            "meshflow=meshflow.cli:main",
        ],
    },
)
