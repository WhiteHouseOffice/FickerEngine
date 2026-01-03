#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

echo "== git status =="
git status --porcelain=v1 -b || true

echo "== git sync =="

# If we're not in a git repo, just build/run
if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Not a git repo -> skipping sync"
else
  branch="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || true)"

  # If upstream exists, sync normally
  if git rev-parse --abbrev-ref --symbolic-full-name '@{u}' >/dev/null 2>&1; then
    echo "Upstream set -> syncing"
    git fetch origin
    git pull --ff-only
  else
    echo "No upstream tracking branch set -> attempting auto-fix"

    if [[ -n "${branch}" && "${branch}" != "HEAD" ]]; then
      if git ls-remote --exit-code --heads origin "${branch}" >/dev/null 2>&1; then
        echo "Setting upstream to origin/${branch}"
        git branch --set-upstream-to="origin/${branch}" "${branch}"
        git fetch origin
        git pull --ff-only
      else
        echo "origin/${branch} does not exist -> cannot sync"
        echo "Push it from Codespaces with: git push -u origin ${branch}"
      fi
    else
      echo "Detached HEAD -> cannot auto-set upstream"
    fi
  fi
fi

# ---- build config ----
BUILD_DIR="${ROOT_DIR}/build"
GENERATOR="${GENERATOR:-Ninja}"

echo "== configure =="
cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -G "$GENERATOR"

echo "== build =="
cmake --build "$BUILD_DIR" -j

echo "== find executable =="

# Try common executable paths/names
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

# Fall back: search for a runnable ELF in build root (fast, shallow)
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
