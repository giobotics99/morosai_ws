from setuptools import setup, find_packages

setup(
    name="morosai_metrics",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "typer",
        "rich",
        "matplotlib",
        "numpy",
        "scipy",
    ],
    entry_points={
        "console_scripts": [
            "morosai_metrics=morosai_metrics.cli.main:app",
        ],
    },
    author="Morosai Team",
    description="A toolkit for analyzing and visualizing ROS2 navigation behavior for the Morosai project.",
    python_requires=">=3.8",
)
