#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys

EXCLUDE_DIRS = {"__pycache__", "build", "icons", "images"}

def should_include(d):
    return (
        not d.startswith('.') and
        d not in EXCLUDE_DIRS
    )

def pack_dir(root_dir: str, out_path: str):
    root_dir = os.path.abspath(root_dir)
    with open(out_path, "w", encoding="utf-8", newline="\n") as out:
        for dirpath, dirnames, filenames in os.walk(root_dir):
            # directories filteren
            dirnames[:] = [d for d in dirnames if should_include(d)]

            for fname in filenames:
                full_path = os.path.join(dirpath, fname)
                rel_path = os.path.relpath(full_path, root_dir)

                try:
                    with open(full_path, "r", encoding="utf-8") as f:
                        content = f.read()
                except UnicodeDecodeError:
                    # binaire files overslaan
                    continue

                out.write(f"FILE_BEGIN {rel_path}\n")
                out.write(content.rstrip("\n") + "\n")
                out.write("FILE_END\n")

    print(f"✅ JBF-bundle geschreven naar: {out_path}")

def main():
    if len(sys.argv) != 3:
        print("Gebruik:")
        print(f"  python3 {sys.argv[0]} <root_dir> <bundle.txt>")
        print("\nVoorbeeld:")
        print(f"  python3 {sys.argv[0]} ./chatGPT project_bundle.txt")
        sys.exit(1)

    root_dir = sys.argv[1]
    out_path = sys.argv[2]

    if not os.path.isdir(root_dir):
        print(f"❌ Directory bestaat niet: {root_dir}")
        sys.exit(1)

    pack_dir(root_dir, out_path)

if __name__ == "__main__":
    main()
