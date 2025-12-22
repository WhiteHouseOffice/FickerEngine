#!/usr/bin/env bash
set -euo pipefail

echo "== git status =="
git status -sb || true

echo "== git pull =="
git pull

echo "== configure =="
cmake -S . -B build -G Ninja

echo "== build =="
cmake --build build -j

echo "== find executable =="
if [[ ! -x build/FickerEngine ]]; then
  echo "No executable at build/FickerEngine"
  exit 1
fi

echo "== run =="
# Only run if we have a display (X11/Wayland). Codespaces usually doesn't.
if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
  echo "▶ build/FickerEngine"
  ./build/FickerEngine
else
  echo "No DISPLAY/WAYLAND_DISPLAY detected -> skipping run (build succeeded)."
fi
