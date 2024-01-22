from setuptools import find_packages, setup

package_name = "py_pubsub"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="triyan",
    maintainer_email="triyanmukherjee@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "publisher = py_pubsub.publisher:main",
            "subscriber = py_pubsub.subscriber:main",
            "snowman=py_pubsub.snowman_maker:main",
        ],
    },
)
