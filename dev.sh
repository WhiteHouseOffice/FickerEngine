#!/usr/bin/env bash
set -euo pipefail

echo "== git status =="
git status -sb || true

echo "== git sync =="
git fetch origin --prune

# If we're on a branch with an upstream, force local to match it.
if git rev-parse --abbrev-ref --symbolic-full-name '@{u}' >/dev/null 2>&1; then
  UPSTREAM="$(git rev-parse --abbrev-ref --symbolic-full-name '@{u}')"
  echo "Upstream: $UPSTREAM"
  git reset --hard "$UPSTREAM"
else
  echo "No upstream tracking branch set -> skipping sync"
fi


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
