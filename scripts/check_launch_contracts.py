#!/usr/bin/env python3
from __future__ import annotations

import ast
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "ros2_ws" / "src"


@dataclass(frozen=True)
class NodeUse:
    package: str
    executable: str
    line: int


@dataclass(frozen=True)
class LaunchContract:
    path: Path
    owner_package: str
    node_uses: list[NodeUse]
    share_packages: list[tuple[str, int]]
    included_launch_names: list[tuple[str, int]]
    enable_torque_defaults: list[tuple[str, int]]


def _constant_string(node: ast.AST | None) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _call_name(node: ast.AST) -> str | None:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return node.attr
    return None


class LaunchVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.node_uses: list[NodeUse] = []
        self.share_packages: list[tuple[str, int]] = []
        self.included_launch_names: list[tuple[str, int]] = []
        self.enable_torque_defaults: list[tuple[str, int]] = []

    def visit_Call(self, node: ast.Call) -> None:
        name = _call_name(node.func)
        if name == "Node":
            self._visit_node_call(node)
        elif name == "get_package_share_directory":
            self._visit_share_call(node)
        elif name == "DeclareLaunchArgument":
            self._visit_launch_argument(node)
        self.generic_visit(node)

    def visit_Constant(self, node: ast.Constant) -> None:
        if isinstance(node.value, str) and node.value.endswith(".launch.py"):
            self.included_launch_names.append((node.value, node.lineno))

    def _visit_node_call(self, node: ast.Call) -> None:
        package = None
        executable = None
        for keyword in node.keywords:
            if keyword.arg == "package":
                package = _constant_string(keyword.value)
            elif keyword.arg == "executable":
                executable = _constant_string(keyword.value)
        if package is not None and executable is not None:
            self.node_uses.append(NodeUse(package, executable, node.lineno))

    def _visit_share_call(self, node: ast.Call) -> None:
        if not node.args:
            return
        package = _constant_string(node.args[0])
        if package is not None:
            self.share_packages.append((package, node.lineno))

    def _visit_launch_argument(self, node: ast.Call) -> None:
        if not node.args:
            return
        argument_name = _constant_string(node.args[0])
        if argument_name != "enable_torque":
            return
        default = None
        for keyword in node.keywords:
            if keyword.arg == "default_value":
                default = _constant_string(keyword.value)
        self.enable_torque_defaults.append((str(default), node.lineno))


def _package_name(package_xml: Path) -> str:
    tree = ET.parse(package_xml)
    name = tree.findtext("name")
    if not name:
        raise ValueError(f"{package_xml} is missing <name>")
    return name


def package_dirs() -> dict[str, Path]:
    dirs = {}
    for package_xml in SRC.glob("*/package.xml"):
        dirs[_package_name(package_xml)] = package_xml.parent
    return dirs


def runtime_dependencies(package_dir: Path) -> set[str]:
    tree = ET.parse(package_dir / "package.xml")
    deps = set()
    for tag in ["depend", "exec_depend"]:
        for element in tree.findall(tag):
            if element.text:
                deps.add(element.text.strip())
    return deps


def console_scripts(package_dir: Path) -> set[str]:
    setup_py = package_dir / "setup.py"
    if not setup_py.exists():
        return set()
    tree = ast.parse(setup_py.read_text(), filename=str(setup_py))
    scripts: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        for keyword in node.keywords:
            if keyword.arg != "entry_points" or not isinstance(keyword.value, ast.Dict):
                continue
            for key, value in zip(keyword.value.keys, keyword.value.values):
                if _constant_string(key) != "console_scripts":
                    continue
                if not isinstance(value, (ast.List, ast.Tuple)):
                    continue
                for item in value.elts:
                    script = _constant_string(item)
                    if script and "=" in script:
                        scripts.add(script.split("=", maxsplit=1)[0].strip())
    return scripts


def launch_contract(path: Path) -> LaunchContract:
    owner = path.relative_to(SRC).parts[0]
    visitor = LaunchVisitor()
    visitor.visit(ast.parse(path.read_text(), filename=str(path)))
    return LaunchContract(
        path=path,
        owner_package=owner,
        node_uses=visitor.node_uses,
        share_packages=visitor.share_packages,
        included_launch_names=visitor.included_launch_names,
        enable_torque_defaults=visitor.enable_torque_defaults,
    )


def validate_contract(contract: LaunchContract, packages: dict[str, Path]) -> list[str]:
    errors: list[str] = []
    owner_dir = packages[contract.owner_package]
    owner_deps = runtime_dependencies(owner_dir)
    scripts_by_package = {
        package: console_scripts(package_dir) for package, package_dir in packages.items()
    }

    referenced_packages = {
        package for package, _ in contract.share_packages if package != contract.owner_package
    }
    referenced_packages.update(node.package for node in contract.node_uses)

    for package in sorted(referenced_packages):
        if package == contract.owner_package:
            continue
        if package not in owner_deps:
            errors.append(
                f"{contract.path}: package {package!r} is referenced but not declared "
                f"as a runtime dependency of {contract.owner_package}"
            )

    for node in contract.node_uses:
        if node.package in packages:
            scripts = scripts_by_package[node.package]
            if node.executable not in scripts:
                errors.append(
                    f"{contract.path}:{node.line}: {node.package!r} does not export "
                    f"console script {node.executable!r}"
                )

    for launch_name, line in contract.included_launch_names:
        matches = list(SRC.glob(f"*/launch/{launch_name}"))
        if not matches:
            errors.append(f"{contract.path}:{line}: included launch {launch_name!r} not found")

    for default, line in contract.enable_torque_defaults:
        if default != "false":
            errors.append(
                f"{contract.path}:{line}: enable_torque default must stay 'false' "
                f"(got {default!r})"
            )

    return errors


def main() -> int:
    packages = package_dirs()
    errors: list[str] = []
    launch_files = sorted(SRC.glob("*/launch/*.py"))
    for path in launch_files:
        contract = launch_contract(path)
        errors.extend(validate_contract(contract, packages))
        print(f"{path.relative_to(ROOT)}: ok")

    if errors:
        print("\nLaunch contract errors", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

