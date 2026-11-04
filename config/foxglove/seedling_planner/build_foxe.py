#!/usr/bin/env python3
"""Package the Seedling Planner Foxglove extension (no Node/npm needed).

    python3 build_foxe.py            # writes canopy-seedling-planner-<version>.foxe here
    python3 build_foxe.py --install  # also installs into ~/.foxglove-studio/extensions

A .foxe is a zip with package.json and the file named by its "main". To install
on another computer, copy the .foxe over and drag it onto the Foxglove window
(or Settings > Extensions > Install local extension), or run this script there.
Restart Foxglove (or reload the layout) after installing.
"""
import argparse
import json
import os
import shutil
import zipfile

HERE = os.path.dirname(os.path.realpath(__file__))
FILES = ["package.json", "extension.js", "README.md"]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--install", action="store_true", help="install into ~/.foxglove-studio/extensions")
    args = parser.parse_args()

    with open(os.path.join(HERE, "package.json")) as f:
        pkg = json.load(f)
    version = pkg["version"]
    out = os.path.join(HERE, f"{pkg['name']}-{version}.foxe")

    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for name in FILES:
            z.write(os.path.join(HERE, name), name)
    print(f"Wrote {out}")

    if args.install:
        ext_root = os.path.expanduser("~/.foxglove-studio/extensions")
        # Foxglove names the folder <publisher>.<name>-<version>; drop older versions.
        prefix = f"{pkg['publisher']}.{pkg['name']}-".lower()
        if os.path.isdir(ext_root):
            for entry in os.listdir(ext_root):
                if entry.lower().startswith(prefix):
                    shutil.rmtree(os.path.join(ext_root, entry))
        dest = os.path.join(ext_root, f"{prefix}{version}")
        os.makedirs(dest, exist_ok=True)
        for name in FILES:
            shutil.copy2(os.path.join(HERE, name), os.path.join(dest, name))
        print(f"Installed to {dest}. Restart Foxglove to load it.")


if __name__ == "__main__":
    main()
