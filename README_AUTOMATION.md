# FickerEngine – Repo Agent Quickstart

This adds a **ChatOps agent** so you can open a GitHub Issue (or comment `/apply`) and get an **automatic PR** with code changes — no local `git` required.

## One-time setup
1. Go to **Settings → Secrets and variables → Actions → New repository secret** and add:
   - `OPENAI_API_KEY`: your OpenAI API key.
2. (Optional) Add a repository variable **`REPO_AGENT_MODEL`** (e.g. `gpt-4o`).
3. Commit the files in this zip at the same paths in your repo.

## How to use
- Open an Issue with a clear title and body describing the change you want, or comment `/apply` on an existing Issue.
- The **Repo Agent** workflow will run, edit files, commit to a new branch, and open a **Pull Request**.

> Tip: Your first Issue can be “Bootstrap WebGPU engine skeleton (CMake + Web target + PWA hooks)” with the details we discussed. The agent will create all the source files and CI needed.
