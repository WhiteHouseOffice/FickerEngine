# FickerEngine — OpenGL Bootstrap

## Repo structure
See `docs/FOLDER_STRUCTURE.md`.

## Physics
See `docs/PHYSICS.md`.

---

## Quick Start (Linux)

Run the development script:

```bash
./dev.sh

This will:

    pull latest changes from the active branch

    build the engine

    run the executable

Branch Workflow

We mainly work on one active development branch and use other branches as failsafe restore points.
Check current branch

git branch

The active branch has a * in front of it.
Switch to active development branch

git checkout dev/physics-fix

If the branch only exists on GitHub:

git fetch origin
git checkout -b dev/physics-fix origin/dev/physics-fix

Pull latest changes

git pull

Push the lates changes in Codespaces 

git add README.md
git commit -m "README: add branch workflow and linux dependencies"
git push





Linux Dependencies (Debian / Ubuntu)

Install required packages:

sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  ninja-build \
  libglfw3-dev \
  libglew-dev \
  libx11-dev \
  libxrandr-dev \
  libxi-dev \
  libxxf86vm-dev \
  libxcursor-dev \
  libxinerama-dev

Optional – Web Build (Emscripten)

sudo apt install emscripten

Check version:

emcc -v

Manual Native Build (Linux)

cmake -S . -B build -G Ninja
cmake --build build
./build/FickerEngine

Useful Git Commands

git branch        # show current branch
git checkout X    # switch branch
git pull          # update current branch
git log --oneline # recent commits

Notes

    Branches are mainly failsafes / restore points

    Active work usually happens on one dev branch

    dev.sh is the fast path for local sync + build + run

Old / Legacy Notes

(Keep any historical notes, experiments, or previous instructions below this line if needed.)