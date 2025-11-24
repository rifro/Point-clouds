#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import tarfile
import gzip

EXCLUDE_DIRS = {"__pycache__", "build", "icons", "images"}
ALLOWED_EXTS = {".cpp", ".h", ".cu", ".cuh", ".txt", ".md", ".sh"}

def should_include_dir(d):
    return not (d.startswith('.') or d in EXCLUDE_DIRS)

def should_include_file(fname):
    _, ext = os.path.splitext(fname.lower())
    return ext in ALLOWED_EXTS

def pack_dir(root_dir: str, out_tar: str):
    root_dir = os.path.abspath(root_dir)
    with tarfile.open(out_tar, "w:gz") as tar:
        for dirpath, dirnames, filenames in os.walk(root_dir):
            # Filter dirs in-place
            dirnames[:] = [d for d in dirnames if should_include_dir(d)]
            rel_dir = os.path.relpath(dirpath, root_dir)
            for fname in filenames:
                if not should_include_file(fname):
                    continue
                full_path = os.path.join(dirpath, fname)
                arcname = os.path.join(rel_dir, fname) if rel_dir != '.' else fname
                try:
                    tar.add(full_path, arcname=arcname)
                except Exception as e:
                    print(f"Skip {full_path}: {e}", file=sys.stderr)

if __name__ == "__main__":
    root = "projects/C++/PluginAutoFit"  # Adjust if needed
    out = "filtered_autofit.tar.gz"
    pack_dir(root, out)
    print(f"Filtered tar.gz created: {out} (allowed: cpp/h/cu/cuh/txt/md/sh, excluded dirs)")
