import os

import toml
from setuptools import setup

# Obtain the extension data from the extension.toml file
EXTENSION_PATH = os.path.dirname(os.path.realpath(__file__))
# Read the extension.toml file
EXTENSION_TOML_DATA = toml.load(
    os.path.join(EXTENSION_PATH, "config", "extension.toml")
)
# Minimum dependencies required prior to installation
INSTALL_REQUIRES = [
    # generic
    "toml",
    "numpy",
]

# Installation operation
setup(
    name="oc_pg_foundationpose_scene",
    author="Titiwat Munintravong (OmniCraft)",
    maintainer="Titiwat Munintravong (OmniCraft)",
    maintainer_email="",
    url=EXTENSION_TOML_DATA["package"]["repository"],
    version=EXTENSION_TOML_DATA["package"]["version"],
    description=EXTENSION_TOML_DATA["package"]["description"],
    keywords=EXTENSION_TOML_DATA["package"]["keywords"],
    license="MIT",
    include_package_data=True,
    python_requires=">=3.10",
    install_requires=INSTALL_REQUIRES,
    packages=[
        "oc.pg.foundationpose_scene",
    ],
    zip_safe=False,
)
