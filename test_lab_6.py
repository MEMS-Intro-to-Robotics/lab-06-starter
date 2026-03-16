#!/usr/bin/env python3
"""
Autograding validation for Lab 6: Pick-and-Place Block Stacking with MoveIt 2.
Run with: pytest test_lab_6.py -v
"""

import ast
import os
import warnings

IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".gif", ".bmp", ".webp"}
EXPECTED_SCREENSHOTS = {
    "01_environment_setup",
    "02_single_stack",
    "03_final_stack_gazebo",
    "04_final_stack_rviz",
}
PKG_DIR = os.path.join("ros2_ws", "src", "lab06_moveit")

README_STARTER_FINGERPRINT = "Update this README"


def _find_in_package(name):
    for root, dirs, files in os.walk(PKG_DIR):
        if name in files:
            return os.path.join(root, name)
    return os.path.join(PKG_DIR, name)


def _get_images(docs_dir="docs"):
    if not os.path.isdir(docs_dir):
        return []
    return [
        f for f in os.listdir(docs_dir)
        if os.path.splitext(f)[1].lower() in IMAGE_EXTENSIONS
    ]


def _check_syntax(path):
    with open(path, "r", errors="replace") as f:
        source = f.read()
    ast.parse(source, filename=path)


def _read_source(path):
    """Read source file and return (source_text, ast_tree) or (None, None)."""
    if not os.path.isfile(path):
        return None, None
    with open(path, "r", errors="replace") as f:
        source = f.read()
    try:
        tree = ast.parse(source, filename=path)
    except SyntaxError:
        return source, None
    return source, tree


def _get_function_names(tree):
    """Extract all function/method definition names from an AST."""
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            names.add(node.name)
    return names


def _get_attribute_calls(tree):
    """Extract all method call names like obj.method() from an AST."""
    calls = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            calls.add(node.func.attr)
    return calls


def _get_string_literals(tree):
    """Extract all string literals from an AST."""
    strings = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            strings.add(node.value)
    return strings


# ── Required files (hard fail) ──────────────────────────────


def test_readme_exists():
    assert os.path.isfile("README.md"), "README.md not found"


def test_readme_not_empty():
    assert os.path.getsize("README.md") > 0, "README.md is empty"


def test_readme_updated():
    with open("README.md", "r", errors="replace") as f:
        content = f.read()
    assert README_STARTER_FINGERPRINT not in content, (
        "README.md still contains the starter template text. "
        "Please update it with your name, NetID, and instructions for running your code."
    )


def test_docs_directory_exists():
    assert os.path.isdir("docs"), "docs/ directory not found"


def test_package_directory_exists():
    assert os.path.isdir(PKG_DIR), f"ROS 2 package not found at {PKG_DIR}"


def test_setup_py_exists():
    assert os.path.isfile(os.path.join(PKG_DIR, "setup.py")), "setup.py not found"


def test_package_xml_exists():
    assert os.path.isfile(os.path.join(PKG_DIR, "package.xml")), "package.xml not found"


def test_pick_and_place_exists():
    path = _find_in_package("pick_and_place.py")
    assert os.path.isfile(path), "pick_and_place.py not found in package"


# ── Python syntax (hard fail) ───────────────────────────────


def test_pick_and_place_syntax():
    _check_syntax(_find_in_package("pick_and_place.py"))


# ── Code structure checks (hard fail) ───────────────────────


def test_has_task_stack_all():
    """Students must implement a task_stack_all function for 3-block stacking."""
    source, tree = _read_source(_find_in_package("pick_and_place.py"))
    assert tree is not None, "Could not parse pick_and_place.py"
    funcs = _get_function_names(tree)
    assert "task_stack_all" in funcs, (
        "Missing function 'task_stack_all'. "
        "You must implement a function that stacks all 3 blocks."
    )


def test_has_main_function():
    source, tree = _read_source(_find_in_package("pick_and_place.py"))
    assert tree is not None, "Could not parse pick_and_place.py"
    funcs = _get_function_names(tree)
    assert "main" in funcs, "Missing main() function"


def test_has_attach_collision_object():
    """Attach/detach is the key MoveIt concept for this lab."""
    source, tree = _read_source(_find_in_package("pick_and_place.py"))
    assert source is not None, "Could not read pick_and_place.py"
    assert "attach_collision_object" in source, (
        "Missing call to 'attach_collision_object'. "
        "You must attach the block to the robot after grasping so the planner "
        "treats it as part of the robot."
    )


def test_has_detach_collision_object():
    source, tree = _read_source(_find_in_package("pick_and_place.py"))
    assert source is not None, "Could not read pick_and_place.py"
    assert "detach_collision_object" in source, (
        "Missing call to 'detach_collision_object'. "
        "You must detach the block from the robot after placing."
    )


def test_has_gripper_operations():
    """Must open and close gripper."""
    source, tree = _read_source(_find_in_package("pick_and_place.py"))
    assert source is not None, "Could not read pick_and_place.py"
    source_lower = source.lower()
    has_open = "open_gripper" in source_lower or "gripper.open" in source_lower or "open(" in source_lower
    has_close = "close_gripper" in source_lower or "gripper.close" in source_lower or "close(" in source_lower
    assert has_open, (
        "No gripper open operation found. "
        "You must open the gripper before grasping and after placing."
    )
    assert has_close, (
        "No gripper close operation found. "
        "You must close the gripper to grasp the block."
    )


def test_has_pose_definitions():
    """Must define approach/grasp/place poses."""
    source, tree = _read_source(_find_in_package("pick_and_place.py"))
    assert source is not None, "Could not read pick_and_place.py"
    source_lower = source.lower()
    pose_keywords = ["pre_grasp", "grasp_pose", "pre_place", "place_pose"]
    found = [kw for kw in pose_keywords if kw in source_lower]
    assert len(found) >= 2, (
        f"Expected pose definitions (pre_grasp, grasp_pose, pre_place, place_pose). "
        f"Only found: {found or 'none'}. "
        "You need approach and contact poses for both grasping and placing."
    )


def test_stacking_uses_loop():
    """task_stack_all should use a loop for the 3-block sequence."""
    source, tree = _read_source(_find_in_package("pick_and_place.py"))
    assert tree is not None, "Could not parse pick_and_place.py"
    # Find the task_stack_all function and check for a loop inside it
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if node.name == "task_stack_all":
                has_loop = any(
                    isinstance(child, (ast.For, ast.While))
                    for child in ast.walk(node)
                )
                assert has_loop, (
                    "task_stack_all does not contain a loop. "
                    "Use a for loop to iterate over the 3 blocks rather than "
                    "copy-pasting the pick-place sequence."
                )
                return
    # If we get here, task_stack_all wasn't found (covered by other test)


def test_has_block_coordinates():
    """Must define coordinates for 3 blocks."""
    source, tree = _read_source(_find_in_package("pick_and_place.py"))
    assert source is not None, "Could not read pick_and_place.py"
    source_lower = source.lower()
    has_blocks = (
        ("block_1" in source_lower or "block1" in source_lower)
        and ("block_2" in source_lower or "block2" in source_lower)
        and ("block_3" in source_lower or "block3" in source_lower)
    ) or "blocks_xyz" in source_lower or "block_centers" in source_lower
    assert has_blocks, (
        "Could not find block coordinate definitions. "
        "You need to define positions for block_1, block_2, and block_3."
    )


def test_has_sleep_after_gripper():
    """Gripper actions need time.sleep() pauses."""
    source, tree = _read_source(_find_in_package("pick_and_place.py"))
    assert source is not None, "Could not read pick_and_place.py"
    assert "sleep" in source, (
        "No time.sleep() calls found. "
        "You need brief pauses after gripper open/close to allow the action to complete."
    )


# ── Screenshots (warnings) ──────────────────────────────────


def test_screenshot_count():
    images = _get_images()
    print(f"\nFound {len(images)} image(s) in docs/:")
    for img in sorted(images):
        print(f"  - {img}")
    if len(images) < 4:
        warnings.warn(f"Expected at least 4 screenshots, found {len(images)}")


def test_screenshot_names():
    images = _get_images()
    stems = {os.path.splitext(f)[0].lower() for f in images}
    missing = EXPECTED_SCREENSHOTS - stems
    if missing:
        warnings.warn(
            f"Screenshots with non-standard names. "
            f"Expected names not found: {', '.join(sorted(missing))}"
        )


# ── Git hygiene (warnings) ──────────────────────────────────


def test_gitignore_exists():
    if not os.path.isfile(".gitignore"):
        warnings.warn(".gitignore not found — build/, install/, log/ should be excluded")


def test_no_build_artifacts():
    for d in ["build", "install", "log"]:
        for base in [".", "ros2_ws"]:
            path = os.path.join(base, d)
            if os.path.isdir(path):
                warnings.warn(f"'{path}' is committed — should be in .gitignore")
