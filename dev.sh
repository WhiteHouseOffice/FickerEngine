#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

BUILD_DIR="${BUILD_DIR:-build}"
GENERATOR="${GENERATOR:-Ninja}"
BUILD_TYPE="${BUILD_TYPE:-RelWithDebInfo}"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 8)}"

echo "== git status =="
git status -sb || true

echo "== git pull =="
git pull --rebase --autostash || true

echo "== configure =="
cmake -S . -B "$BUILD_DIR" -G "$GENERATOR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

echo "== build =="
cmake --build "$BUILD_DIR" -j "$JOBS"

echo "== find executable =="
EXE="$(find "$BUILD_DIR" -maxdepth 3 -type f -executable | head -n 1)"

if [[ -z "$EXE" ]]; then
  echo "❌ No executable found in $BUILD_DIR"
  exit 1
fi

echo "== run =="
echo "▶ $EXE"
exec "$EXE"
