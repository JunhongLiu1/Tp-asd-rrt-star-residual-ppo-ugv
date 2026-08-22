import os
from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = "risk_aware_residual_rl"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml", "README.md", "requirements-ppo-py38.txt"],
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.json"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="i",
    maintainer_email="i@todo.todo",
    description=(
        "Safety-bounded residual control core and optional PPO tooling."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "residual_ppo_train = "
            "risk_aware_residual_rl.training:main",
            "residual_ppo_evaluate = "
            "risk_aware_residual_rl.evaluation:main",
            "residual_ppo_finalize = "
            "risk_aware_residual_rl.artifact_finalize:main",
            "residual_policy_node = "
            "risk_aware_residual_rl.residual_policy_node:main",
        ],
    },
)
