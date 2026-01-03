#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

# Which branch should dev.sh sync to by default?
# You can override per-run:
#   FE_BRANCH=work/physics-next ./dev.sh
BRANCH="${FE_BRANCH:-dev/jump-physics}"

echo "== repo =="
echo "cwd: $ROOT"

echo "== git status (before) =="
git status -sb || true

echo "== sync (fetch + checkout + pull) =="
# Only attempt branch sync if working tree is clean.
if ! git diff --quiet || ! git diff --cached --quiet; then
  echo "(skip) working tree has local changes; not switching branches or pulling."
else
  git fetch origin --prune

  # If the remote branch exists, force local branch to track it.
  if git show-ref --verify --quiet "refs/remotes/origin/$BRANCH"; then
    # Create/switch local branch to track origin/<BRANCH>
    git checkout -B "$BRANCH" "origin/$BRANCH"
    git pull --ff-only
  else
    echo "(warn) remote branch origin/$BRANCH not found; skipping pull."
  fi
fi

echo "== git status (after) =="
git status -sb || true

echo "== configure =="
rm -rf build
cmake -S . -B build -G Ninja

echo "== build =="
cmake --build build -j

echo "== run =="
./build/FickerEngine
