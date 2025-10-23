#!/usr/bin/env python

# Increases the version number of package.xml and CMakeLists.txt files in
# subfolders. The first argument specifies the version increase:
# major, minor, or patch (default, e.g. 1.6.2 --> 1.6.3)
#
# Borrows heaviliy from ROS / catkin release tools

import argparse
import re
import sys
from pathlib import Path

SUBPROJECTS = ["octomap", "octovis", "dynamicEDT3D"]
DEFAULT_ROOTDIR = Path(__file__).parent.parent

parser = argparse.ArgumentParser()
parser.add_argument(
    "--rootdir",
    metavar="DIR",
    type=Path,
    default=DEFAULT_ROOTDIR,
    help=f"Octomap source tree (default: {DEFAULT_ROOTDIR})",
)
m = parser.add_mutually_exclusive_group()
m.add_argument(
    "--bump",
    choices=["major", "minor", "patch"],
    default="patch",
    help="Which part of the version number to bump? (default: patch)",
)
m.add_argument(
    "--version", metavar="MAJOR.MINOR.PATCH", help="Set a specific version to use"
)

args = parser.parse_args()

MANIFESTS = [args.rootdir / p / "package.xml" for p in SUBPROJECTS]
CMAKELISTS = [args.rootdir / p / "CMakeLists.txt" for p in SUBPROJECTS] + [
    args.rootdir / "CMakeLists.txt"
]

missing_files = [str(f) for f in MANIFESTS + CMAKELISTS if not f.is_file()]
if missing_files:
    sys.stderr.write(f"Missing files:\n* {'\n* '.join(missing_files)}\n")
    sys.exit(1)

if args.version is None:
    # Verify that all versions are consistent before bumping
    versions = set()
    for manifest in MANIFESTS:
        with open(manifest, "r") as f:
            content = f.read()
        mo = re.search(r"<version>(\d)+\.(\d+)\.(\d+)</version>", content)
        if not mo:
            sys.stderr.write(f"Cannot find <version> tag in {manifest}\n")
            sys.exit(1)
        versions.add((int(mo.group(1)), int(mo.group(2)), int(mo.group(3))))
    for cmakelist in CMAKELISTS:
        with open(cmakelist, "r") as f:
            content = f.read()
        mo = re.search(r"project\s*\([^)]+VERSION\s+(\d)+\.(\d+)\.(\d+)", content)
        if not mo:
            sys.stderr.write(f"Cannot find project VERSION option in {cmakelist}\n")
            sys.exit(1)
        versions.add((int(mo.group(1)), int(mo.group(2)), int(mo.group(3))))
    if len(versions) != 1:
        sys.stderr.write(
            f"Cannot bump multiple inconsistent versions: {', '.join('.'.join(str(v) for v in version) for version in sorted(versions))}\n"
        )
    version_numbers = [v for v in list(versions)[0]]
    if args.bump == "patch":
        version_numbers[2] += 1
    if args.bump == "minor":
        version_numbers[1] += 1
        version_numbers[2] = 0
    if args.bump == "major":
        version_numbers[0] += 1
        version_numbers[1] = 0
        version_numbers[2] = 0
    args.version = ".".join(str(v) for v in version_numbers)
elif not re.match(r"^(\d+)\.(\d+)\.(\d+)$", args.version):
    sys.stderr.write(f"Version {args.version!r} is not in MAJOR.MINOR.PATCH format\n")
    sys.exit(1)

anything_changed = False

for manifest in MANIFESTS:
    with open(manifest, "r") as f:
        old_content = f.read()
    new_content = re.sub(
        r"<version>[0-9.]+</version>", f"<version>{args.version}</version>", old_content
    )
    if old_content != new_content:
        anything_changed = True
        sys.stdout.write(f"Updating {manifest} ...\n")
        with open(manifest, "w") as f:
            f.write(new_content)

for cmakelist in CMAKELISTS:
    with open(cmakelist, "r") as f:
        old_content = f.read()
    new_content = re.sub(
        r"(project\s*\([^)]+)VERSION\s+[0-9.]+",
        f"\\1VERSION {args.version}",
        old_content,
    )
    if old_content != new_content:
        anything_changed = True
        sys.stdout.write(f"Updating {cmakelist} ...\n")
        with open(cmakelist, "w") as f:
            f.write(new_content)

if not anything_changed:
    sys.stdout.write("Nothing to update.\n")
    sys.exit(0)

sys.stdout.write(
    f"""\n
Finished writing package.xml and CMakeLists.txt files.
Now check the output, adjust CHANGELOG, and "git commit".
Finally, run:
  git checkout master && git merge --no-ff devel && git tag v{args.version}
  git push origin master devel && git push origin --tags
  (adjust if not on the "devel" branch)

"""
)
sys.exit(0)
