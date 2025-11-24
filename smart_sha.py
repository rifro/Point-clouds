#!/usr/bin/env python3
import sys
import hashlib
import re

def normalize_code(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        code = f.read()

    # Remove comments (// and /* */)
    code = re.sub(r'//.*', '', code)  # Single-line
    code = re.sub(r'/\*.*?\*/', '', code, flags=re.DOTALL)  # Multi-line

    # Normalize whitespace: Collapse multiple WS to single space, trim lines, no trailing \n
    code = re.sub(r'\s+', ' ', code)  # Replace all WS with single space
    code = re.sub(r'\n\s*\n', '\n', code)  # Collapse empty lines
    code = code.strip()  # Trim overall

    return code

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 smart_sha.py <file.cpp>")
        sys.exit(1)
    file_path = sys.argv[1]
    normalized = normalize_code(file_path)
    sha = hashlib.sha256(normalized.encode('utf-8')).hexdigest()
    print(f"Normalized SHA for {file_path}: {sha}")
