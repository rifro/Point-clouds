#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import os
import sys

def strip_outer_fence(text):
    """
    Verwijder exact één outer ``` fence als die het hele document omsluit.
    (Markdown copy-button probleem oplossen)
    """
    lines = text.splitlines()

    if len(lines) >= 2 and lines[0].strip() == "```" and lines[-1].strip() == "```":
        return "\n".join(lines[1:-1])
    return text


def split_bundle(bundle_text):
    """
    Splitst FILE_BEGIN/FILE_END of <file></file>
    in losse bestanden, zonder ooit inhoud te wijzigen.
    """

    pattern = re.compile(
        r'(?:FILE_BEGIN\s+([^\s]+)|<file\s+path="([^"]+)">)\s*(.*?)\s*(?:FILE_END|</file>)',
        re.DOTALL
    )

    matches = list(pattern.finditer(bundle_text))

    if not matches:
        print("❌ Geen file-blokken gevonden. Controleer bundel.\n")
        return

    format_used = "FILE_BEGIN/FILE_END" if "FILE_BEGIN" in bundle_text else "<file></file>"
    print(f"✅ {len(matches)} bestanden gevonden ({format_used}-formaat)")

    for m in matches:
        file_path = m.group(1) or m.group(2)
        content = m.group(3)

        if not file_path:
            print("⚠️ Blok zonder bestandsnaam overgeslagen.")
            continue

        directory = os.path.dirname(file_path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)
            print(f"📁 map aangemaakt: {directory}")

        with open(file_path, "w", encoding="utf-8", newline="\n") as f:
            # Schrijf EXACTE inhoud, geen strip of md fix
            f.write(content.rstrip("\n") + "\n")

        print(f"✔️ schreef: {file_path}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Gebruik: python3 splitter_raw.py bundle.txt")
        sys.exit(1)

    with open(sys.argv[1], "r", encoding="utf-8") as f:
        text = f.read()

    text = strip_outer_fence(text)
    split_bundle(text)

