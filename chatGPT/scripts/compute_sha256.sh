#!/usr/bin/env bash
set -euo pipefail
find src include scripts docs -type f -print0 | xargs -0 sha256sum