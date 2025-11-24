#!/usr/bin/env python3
import sys, re, os, hashlib, argparse, pathlib

TAG_RE = re.compile(r'<file path="([^"]+)">\n?(.*?)