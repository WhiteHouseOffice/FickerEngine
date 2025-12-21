#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

# Defaults (override via env)
BUILD_DIR="${BUILD_DIR:-build}"
GENERATOR="${GENERATOR:-Ninja}"
BUILD_TYPE="${BUILD_TYPE:-RelWithDebInfo}"
JOBS="${JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 8)}"

# --- git update ---
echo "== git status =="
git status -sb || true

echo "== git pull =="
# If you're on a branch with upstream set, this will pull; if not, it won't explode.
git pull --rebase --autostash || true

# --- configure ---
echo "== configure =="
cmake -S . -B "$BUILD_DIR" -G "$GENERATOR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

# --- build ---
echo "== build =="
cmake --build "$BUILD_DIR" -j "$JOBS"

echo "✅ Done. Build dir: $BUILD_DIR"
