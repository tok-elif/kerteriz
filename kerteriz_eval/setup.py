from setuptools import setup

package_name = "kerteriz_eval"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "matplotlib"],
    zip_safe=True,
    maintainer="Kerteriz",
    maintainer_email="tokelifw@gmail.com",
    description="Degerlendirme: evo sarmalayici, NEES/NIS, Stanford diyagrami, rapor.",
    license="MIT",
)
