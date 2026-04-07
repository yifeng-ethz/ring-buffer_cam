#!/usr/bin/env python3

import argparse
import xml.etree.ElementTree as ET
from collections import defaultdict


def parse_args():
    parser = argparse.ArgumentParser(
        description="Normalize a Platform Designer components.ipx catalog."
    )
    parser.add_argument("--input", required=True, help="Input components.ipx path")
    parser.add_argument("--output", required=True, help="Output components.ipx path")
    return parser.parse_args()


def score_component(component):
    path = component.get("file", "")
    return (path.count("/"), len(path), path)


def main():
    args = parse_args()
    tree = ET.parse(args.input)
    root = tree.getroot()

    components = [child for child in list(root) if child.tag == "component"]
    best_by_key = {}
    order = []
    versions_by_name = defaultdict(set)

    for component in components:
        name = component.get("name", "")
        version = component.get("version", "")
        key = (name, version)
        versions_by_name[name].add(version)
        if key not in best_by_key:
            best_by_key[key] = component
            order.append(key)
            continue
        if score_component(component) < score_component(best_by_key[key]):
            best_by_key[key] = component

    for child in list(root):
        if child.tag == "component":
            root.remove(child)

    duplicates_removed = len(components) - len(order)
    multi_version_names = {
        name for name, versions in versions_by_name.items() if len(versions) > 1
    }

    for key in order:
        component = best_by_key[key]
        name = component.get("name", "")
        version = component.get("version", "")
        display_name = component.get("displayName", name)
        if name in multi_version_names:
            suffix = f" ({version})"
            if not display_name.endswith(suffix):
                component.set("displayName", f"{display_name}{suffix}")
        root.append(component)

    ET.indent(tree, space=" ", level=0)
    tree.write(args.output, encoding="UTF-8", xml_declaration=True)
    print(
        f"Normalized {args.input}: kept {len(order)} unique components, "
        f"removed {duplicates_removed} duplicate name/version entries, "
        f"version-tagged {len(multi_version_names)} multi-version component families."
    )


if __name__ == "__main__":
    main()
