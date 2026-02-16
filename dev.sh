#!/usr/bin/env bash
set -euo pipefail

# Never allow git to prompt for credentials (prevents blocking dev.sh).
export GIT_TERMINAL_PROMPT=0

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

# ------------------------------------------------------------
# Git sync (simple + non-blocking)
# ------------------------------------------------------------
DEV_BRANCH="dev/physics-fix"   # <- change this when we move to another active branch

if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "== git status =="
  git status --porcelain=v1 -b || true

  # If we're not already on the dev branch, try to switch to it.
  # If it doesn't exist locally, try to create it from origin.
  current_branch="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "")"
  if [[ -n "${DEV_BRANCH}" && "${current_branch}" != "${DEV_BRANCH}" && "${current_branch}" != "HEAD" ]]; then
    if git show-ref --verify --quiet "refs/heads/${DEV_BRANCH}"; then
      git checkout "${DEV_BRANCH}" >/dev/null 2>&1 || true
    else
      git fetch --all --prune >/dev/null 2>&1 || true
      if git show-ref --verify --quiet "refs/remotes/origin/${DEV_BRANCH}"; then
        git checkout -b "${DEV_BRANCH}" "origin/${DEV_BRANCH}" >/dev/null 2>&1 || true
      fi
    fi
  fi

  echo "== git pull =="
  git fetch --all --prune >/dev/null 2>&1 || true
  git pull --ff-only >/dev/null 2>&1 || true
else
  echo "Not a git repo -> skipping sync"
fi

# ------------------------------------------------------------
# Build
# ------------------------------------------------------------
BUILD_DIR="${ROOT_DIR}/build"
GENERATOR="${GENERATOR:-Ninja}"

echo "== configure =="
cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -G "$GENERATOR"

echo "== build =="
cmake --build "$BUILD_DIR" -j

echo "== find executable =="

CANDIDATES=(
  "$BUILD_DIR/FickerEngine"
  "$BUILD_DIR/fickerengine"
  "$BUILD_DIR/bin/FickerEngine"
  "$BUILD_DIR/bin/fickerengine"
)

EXE=""
for c in "${CANDIDATES[@]}"; do
  if [[ -x "$c" ]]; then
    EXE="$c"
    break
  fi
done

if [[ -z "$EXE" ]]; then
  while IFS= read -r -d '' f; do
    EXE="$f"
    break
  done < <(find "$BUILD_DIR" -maxdepth 2 -type f -perm -111 -print0 2>/dev/null || true)
fi

if [[ -z "$EXE" ]]; then
  echo "Could not find executable in build/. Check your CMake output."
  exit 1
fi

echo "== run =="
echo "▶ ${EXE#"$ROOT_DIR/"}"
"$EXE"
