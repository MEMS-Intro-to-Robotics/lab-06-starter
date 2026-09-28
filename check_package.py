#!/usr/bin/env python3
"""Check the Lab 6 package setup and report fixes for detected problems.

Run it on the VM (Host VM Terminal), from the root of your Lab 6 repository,
after you create the package and edit setup.py, and before you build:

    python3 check_package.py

It checks where the package is, what it is called, the files it must contain,
the setup.py entry points, who owns the files, and whether colcon was run from
the wrong folder. It changes nothing. Each reported problem includes the
command or edit that fixes it. When it prints "Package setup looks right",
build in the container as the manual says.

The file and entry-point checks are the same ones the autograder runs.
"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO))

from test_lab_6 import (  # noqa: E402  (needs the path above)
    PACKAGE_ROOT,
    PYTHON_PACKAGE,
    SCRIPTS_PACKAGE,
    check_package,
)

PACKAGE_NAME = PACKAGE_ROOT.name
SKIP = {".git", "build", "install", "log", "__pycache__", ".pytest_cache"}


def _package_xml_files() -> list[Path]:
    found = []
    for root, dirs, files in os.walk(REPO):
        dirs[:] = [d for d in dirs if d not in SKIP]
        if "package.xml" in files:
            found.append(Path(root) / "package.xml")
    return found


def _package_name(package_xml: Path) -> str | None:
    match = re.search(r"<name>\s*([^<\s]+)\s*</name>", package_xml.read_text(errors="replace"))
    return match.group(1) if match else None


def location_problems() -> list[str]:
    """Where the package is, and whether it is called lab06_moveit."""
    expected = REPO / PACKAGE_ROOT
    if (expected / "package.xml").is_file():
        name = _package_name(expected / "package.xml")
        if name != PACKAGE_NAME:
            named = f"names the package '{name}'" if name else "has no <name>"
            return [
                f"package.xml {named}, but it must name '{PACKAGE_NAME}'. "
                f"FIX: delete {PACKAGE_ROOT.as_posix()} and create it again with the "
                "ros2 pkg create command from the manual, spelled exactly."
            ]
        return []
    elsewhere = [p.parent for p in _package_xml_files()]
    if not elsewhere:
        return [
            f"No ROS 2 package found. Expected {PACKAGE_ROOT.as_posix()}/. "
            "FIX: in the container, cd into ros2_ws/src of this repository and run the "
            "ros2 pkg create command from the manual."
        ]
    problems = []
    for folder in elsewhere:
        relative = folder.relative_to(REPO).as_posix()
        name = _package_name(folder / "package.xml")
        if name == PACKAGE_NAME:
            problems.append(
                f"The package is at {relative}/, but it must be at {PACKAGE_ROOT.as_posix()}/. "
                "It was probably created from the wrong folder. "
                f"FIX (Host VM Terminal, repository root): mv {relative} {PACKAGE_ROOT.as_posix()}"
            )
        else:
            problems.append(
                f"Found a package named '{name}' at {relative}/, but the lab needs one named "
                f"'{PACKAGE_NAME}' at {PACKAGE_ROOT.as_posix()}/. FIX: delete {relative} and create "
                "the package again with the command from the manual, spelled exactly."
            )
    return problems


def ownership_problems() -> list[str]:
    """Files the container created as root that VS Code cannot save."""
    if not hasattr(os, "getuid") or os.getuid() == 0:
        return []  # inside the container, everything is root; nothing to check
    root_owned = []
    for root, dirs, files in os.walk(REPO / "ros2_ws" / "src"):
        dirs[:] = [d for d in dirs if d not in SKIP]
        for name in dirs + files:
            path = Path(root) / name
            try:
                if path.lstat().st_uid == 0:
                    root_owned.append(path.relative_to(REPO).as_posix())
            except OSError:
                continue
    if not root_owned:
        return []
    return [
        f"{len(root_owned)} file(s) in ros2_ws/src are owned by root (for example "
        f"{root_owned[0]}), so VS Code cannot save them. "
        f"FIX (Host VM Terminal): sudo chown -R $USER:$USER {REPO}"
    ]


def colcon_problems() -> list[str]:
    """build/, install/, and log/ made by running colcon from the wrong folder."""
    wrong = [
        folder for folder in (REPO, REPO / "ros2_ws" / "src", REPO / PACKAGE_ROOT)
        if any((folder / name).is_dir() for name in ("build", "install", "log"))
    ]
    problems = []
    for folder in wrong:
        relative = folder.relative_to(REPO).as_posix() if folder != REPO else "the repository root"
        where = "" if folder == REPO else f"{folder.relative_to(REPO).as_posix()}/"
        problems.append(
            f"colcon was run from {relative}; it must be run from ros2_ws. "
            f"FIX (Host VM Terminal, repository root): sudo rm -rf {where}build {where}install {where}log "
            "then build again from ros2_ws in the container."
        )
    return problems


def file_problems() -> list[str]:
    """The autograder's package checks, with the likely cause of each."""
    problems = []
    for error in check_package(REPO):
        if "scripts/__init__.py" in error:
            error += (
                f" FIX (Host VM Terminal, repository root): touch {SCRIPTS_PACKAGE.as_posix()}/__init__.py"
            )
        elif "Add the missing required file" in error and "/scripts/" in error:
            error += (
                " FIX (Host VM Terminal, repository root): copy it from scaffolds/ with the cp "
                "command in the manual."
            )
        elif "Add the missing required file" in error and PYTHON_PACKAGE.as_posix() in error:
            error += " ros2 pkg create makes this file; create the package again."
        elif "console_scripts" in error or "setup.py" in error:
            error += " Compare your entry_points block with the one in the manual, character by character."
        problems.append(error)
    return problems


def main() -> int:
    if not (REPO / "test_lab_6.py").is_file() or not (REPO / ".git").exists():
        print("Run this from the root of your Lab 6 repository (the folder with test_lab_6.py).")
        return 2
    problems = location_problems()
    if not problems:
        problems = file_problems()
    problems += ownership_problems() + colcon_problems()
    if not problems:
        print("Package setup looks right. Build it in the container from ros2_ws:")
        print("    colcon build --symlink-install")
        print("Start the simulation from the repository root (the folder with lab06_sim.launch.py),")
        print("in a terminal where you ran source install/setup.bash after that build:")
        print("    ros2 launch lab06_sim.launch.py")
        return 0
    print(f"Found {len(problems)} problem(s) with the package setup. Fix them in order, "
          "then run python3 check_package.py again.\n")
    for number, problem in enumerate(problems, start=1):
        print(f"{number}. {problem}\n")
    return 1


if __name__ == "__main__":
    sys.exit(main())
