# 🧩 FickerEngine – Quick Start (WebGPU / Codespaces Build)

This guide describes how to **build and preview FickerEngine in a GitHub Codespace**.  
It recompiles the C++ engine to WebAssembly using **Emscripten** and runs it in the browser via **WebGPU** (or a 2D fallback if WebGPU isn’t available).

### Purpose
Use this when you want to:
- Build the current engine version inside Codespaces  
- Serve the compiled `engine.js` and `engine.wasm` in a browser  
- Visually verify rendering output (e.g. the triangle test)  

### Steps

1. **Load the Emscripten environment**
   ```bash
   source emsdk/emsdk_env.sh
Configure the Web build (one time per session)

bash
Copy code
emcmake cmake -S . -B webbuild -D BUILD_WEB=ON -G Ninja
Compile

bash
Copy code
cmake --build webbuild
Stage the web output

bash
Copy code
mkdir -p webbuild/site
cp -r web/* webbuild/site/ || true
find webbuild -maxdepth 2 -type f -name "engine.*" -exec cp {} webbuild/site/ \; || true
Serve locally inside Codespaces

bash
Copy code
cd webbuild/site
python3 -m http.server 8080
Open the app

In the Ports panel, find port 8080

Mark it as Public

Click the URL → opens the Web preview

Press stepOnce() to render the test triangle 🎉
# Session 2025-10-21
**Goal:** got first WebGPU triangle building in CI  
**Status:** ✅ success, renders triangle in artifact  
**Next time:** 
- Use “How to view the app in Codespaces (quick refresher)” steps to preview locally  

- Then add `serve.sh` and `.gitattributes`

Reference: see docs/SESSION_GUIDES.md → “View the app in Codespaces”
