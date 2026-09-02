#!/usr/bin/env python3
"""Check every parameter file against the parameters the sources declare.

A ROS 2 node silently ignores a parameter nobody declares, so a typo or a key
left behind by a refactor does nothing and says nothing. This walks the YAML
parameter files, walks the ``declare_and_get`` calls in the sources, and
reports keys that reach neither.

Run directly, or through ``ctest`` as ``validate_config_parameters``.
"""

import argparse
import pathlib
import re
import sys

import yaml

# declare_and_get<T>("name", default). The template argument may itself be a
# template, as in std::vector<double>, so one level of nesting is allowed.
_LITERAL_DECLARATION = re.compile(
    r'declare_and_get\s*<((?:[^<>]|<[^<>]*>)*)>\s*\(\s*"([^"]+)"')

# Inside the shared helper: declare(prefix + "suffix", default)
_PREFIXED_DECLARATION = re.compile(
    r'declare\s*\(\s*prefix\s*\+\s*"([^"]+)"\s*,\s*([^)]+?)\)')

# Call sites that expand that helper. The lambda argument in between contains
# semicolons and nested parentheses, so scan a bounded window after the call
# for the first dotted string literal, which is the prefix.
_DECLARE_CONFIG_CALL = re.compile(r'declareConfig\s*\(')
_PREFIX_LITERAL = re.compile(r'"([A-Za-z0-9_]+(?:\.[A-Za-z0-9_]+)*\.)"')
_PREFIX_WINDOW = 800

# Node-level names that are not under a config namespace.
_ROS_RESERVED = {"use_sim_time"}

# Namespaces belonging to third-party nodes that share these parameter files:
# robot_localization's navsat_transform_node and ekf_node. Their parameters are
# theirs to validate, not ours.
_EXTERNAL_NAMESPACES = {"navsat", "ekf_gps"}


def _yaml_type_of(cpp_type):
    """The YAML type rclcpp will insist on for a declared C++ type.

    rclcpp is strict: a parameter declared double and given an integer in a
    parameter file raises InvalidParameterTypeException at node construction,
    so 1000 and 1000.0 are not interchangeable.
    """
    normalized = cpp_type.replace(" ", "")
    if normalized == "bool":
        return "bool"
    if normalized in {"int", "int64_t", "std::int64_t", "std::size_t"}:
        return "int"
    if normalized in {"double", "float"}:
        return "float"
    if normalized in {"std::string", "string"}:
        return "str"
    if normalized in {"std::vector<double>", "std::vector<float>"}:
        return "float_list"
    return None


def declared_parameters(source_roots):
    """Every parameter name the C++ sources can declare, with its type."""
    literal = {}
    suffixes = {}
    prefixes = set()

    for root in source_roots:
        for path in sorted(root.rglob("*")):
            if path.suffix not in {".cpp", ".hpp", ".h"} or not path.is_file():
                continue
            text = path.read_text(errors="replace")
            for cpp_type, name in _LITERAL_DECLARATION.findall(text):
                literal[name] = _yaml_type_of(cpp_type)
            for name, default in _PREFIXED_DECLARATION.findall(text):
                suffixes[name] = _default_literal_type(default)
            for call in _DECLARE_CONFIG_CALL.finditer(text):
                window = text[call.end():call.end() + _PREFIX_WINDOW]
                match = _PREFIX_LITERAL.search(window)
                if match:
                    prefixes.add(match.group(1))

    declared = dict(literal)
    for prefix in prefixes:
        for suffix, kind in suffixes.items():
            declared[prefix + suffix] = kind
    return declared, literal, suffixes, prefixes


def _default_literal_type(default):
    """Type of a C++ literal default, for the prefixed declarations.

    The shared helper deduces each parameter's type from its default, so the
    default's spelling is the contract: 2.0 is a double, 5000 an int.
    """
    text = default.strip()
    if text in {"true", "false"}:
        return "bool"
    if re.fullmatch(r"-?\d+", text):
        return "int"
    if re.fullmatch(r"-?\d*\.\d+(?:[eE][-+]?\d+)?", text):
        return "float"
    if text.startswith('"'):
        return "str"
    return None


def _observed_type(value):
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, str):
        return "str"
    if isinstance(value, list):
        if all(isinstance(item, bool) for item in value):
            return "bool_list"
        if all(isinstance(item, int) and not isinstance(item, bool)
               for item in value):
            return "int_list"
        if all(isinstance(item, (int, float)) and not isinstance(item, bool)
               for item in value):
            return "float_list"
        if all(isinstance(item, str) for item in value):
            return "str_list"
        return "mixed_list"
    return None


def yaml_parameter_keys(document):
    """Flatten one parameter file into (dotted name, value) pairs."""
    keys = []

    def walk(node, path):
        if isinstance(node, dict):
            for key, value in node.items():
                walk(value, path + [str(key)])
        else:
            keys.append((".".join(path), node))

    for node_name, node_body in (document or {}).items():
        if not isinstance(node_body, dict):
            continue
        parameters = node_body.get("ros__parameters")
        if parameters is None:
            print(f"  note: {node_name} has no ros__parameters block")
            continue
        walk(parameters, [])
    return keys


# Launch files reference share-directory assets as ('config', '<name>').
_LAUNCH_ASSET = re.compile(r"'config',\s*'([^']+)'")


def check_launch_references(repo):
    """Every asset a launch file names must exist somewhere it is installed."""
    launch_root = repo / "launch"
    if not launch_root.is_dir():
        return 0

    search_roots = [
        repo / "config",
        repo / "src" / "liorf-DiSO" / "config",
        repo / "launch" / "include" / "config",
    ]

    failures = 0
    for launch in sorted(launch_root.rglob("*.launch.py")):
        missing = []
        for asset in _LAUNCH_ASSET.findall(launch.read_text()):
            if not any((root / asset).exists() for root in search_roots):
                missing.append(asset)
        status = "ok  " if not missing else "FAIL"
        print(f"{status} {launch.relative_to(repo)}")
        for asset in missing:
            print(f"       missing asset: {asset}")
        failures += bool(missing)
    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo", default=str(pathlib.Path(__file__).resolve().parent.parent),
        help="Repository root (default: the parent of this script's directory)")
    arguments = parser.parse_args()

    repo = pathlib.Path(arguments.repo).resolve()
    source_roots = [repo / "src", repo / "include"]
    config_roots = [repo / "config", repo / "src" / "liorf-DiSO" / "config"]

    declared, literal, suffixes, prefixes = declared_parameters(source_roots)
    if not declared:
        print("FAIL: no parameter declarations found; check --repo")
        return 1

    print(f"Declared parameters: {len(declared)} "
          f"({len(literal)} literal, {len(suffixes)} suffixes "
          f"x {len(prefixes)} prefixes)")

    config_files = []
    for root in config_roots:
        if root.is_dir():
            config_files.extend(sorted(root.glob("*.yaml")))
    if not config_files:
        print("FAIL: no parameter files found")
        return 1

    failures = 0
    for path in config_files:
        try:
            document = yaml.safe_load(path.read_text())
        except yaml.YAMLError as error:
            print(f"FAIL {path.relative_to(repo)}: not valid YAML: {error}")
            failures += 1
            continue

        keys = yaml_parameter_keys(document)
        ours = [(key, value) for key, value in keys
                if key.split(".")[0] not in _EXTERNAL_NAMESPACES]
        external = len(keys) - len(ours)

        problems = []
        for key, value in ours:
            if key in _ROS_RESERVED:
                continue
            if key not in declared:
                problems.append(f"undeclared: {key}")
                continue
            expected = declared[key]
            observed = _observed_type(value)
            if expected is None or observed is None:
                continue
            # An int list is an acceptable spelling of a float list only when
            # every entry is whole, which is how rotation matrices are written.
            if expected == "float_list" and observed == "int_list":
                continue
            if expected != observed:
                problems.append(
                    f"type: {key} is {observed} in the file but declared "
                    f"{expected}; rclcpp rejects this at startup")

        status = "ok  " if not problems else "FAIL"
        summary = f"{len(ours)} keys"
        if external:
            summary += f" (+{external} third-party)"
        print(f"{status} {path.relative_to(repo)}: {summary}")
        for problem in problems:
            print(f"       {problem}")
        failures += bool(problems)

    print()
    failures += check_launch_references(repo)

    if failures:
        print(f"\n{failures} file(s) do not match the contract.")
        print("An undeclared key is ignored silently and has no effect; a "
              "mistyped one stops the node from starting.")
        return 1

    print(f"\nAll {len(config_files)} parameter files match the declared "
          f"parameter contract, and every launch asset resolves.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
