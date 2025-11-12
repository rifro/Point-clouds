#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import os
import sys

def split_bundle(bundle_text):
    """
    Splitst de gebundelde XML-achtige of FILE_BEGIN/FILE_END tekst in afzonderlijke bestanden.
    Ondersteunt beide formaten automatisch.
    """

    # Regex: match FILE_BEGIN or <file>, same for end
    pattern = re.compile(
        r'(?:FILE_BEGIN\s+([^\s]+)|<file\s+path="([^"]+)">)\s*(.*?)\s*(?:FILE_END|</file>)',
        re.DOTALL
    )

    matches = list(pattern.finditer(bundle_text))

    if not matches:
        print("❌ Geen file-blokken gevonden (<file ...> of FILE_BEGIN ...). Controleer het bestand.\n")
        return

    # Detecteer welk type gebruikt is
    if "FILE_BEGIN" in bundle_text:
        format_used = "FILE_BEGIN/FILE_END"
    else:
        format_used = "<file></file>"

    print(f"✅ {len(matches)} bestanden gevonden ({format_used}-formaat), start uitpakken…")

    for m in matches:
        file_path = m.group(1) or m.group(2)
        content   = m.group(3)

        if not file_path:
            print("⚠️  blok zonder bestandsnaam overgeslagen")
            continue

        # Map aanmaken indien nodig
        directory = os.path.dirname(file_path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)
            print(f"📁 map aangemaakt: {directory}")

        # Wegschrijven (UTF-8, Unix line endings)
        try:
            with open(file_path, "w", encoding="utf-8", newline="\n") as f:
                f.write(content.strip() + "\n")
            print(f"✔️  schreef: {file_path}")
        except Exception as e:
            print(f"❌ fout bij schrijven {file_path}: {e}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Gebruik:")
        print(f"  python3 {sys.argv[0]} bundle.txt\n")
        sys.exit(1)

    bundle = sys.argv[1]

    try:
        with open(bundle, "r", encoding="utf-8") as f:
            text = f.read()
    except Exception as e:
        print(f"❌ kan bundle niet lezen: {e}")
        sys.exit(1)

    split_bundle(text)
    print("\n🎉 Splitsing voltooid.")
