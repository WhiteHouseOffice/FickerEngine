# scripts/agent/repo_agent.py
import os
import sys
import json
import subprocess
import textwrap
from pathlib import Path, PurePosixPath
from datetime import datetime

# --- OpenAI client ---
# (Workflow installs: httpx==0.27.2 openai==1.42.0)
from openai import OpenAI

# -------------- Guardrails & helpers --------------

# Directories the agent IS allowed to modify (top-level)
ALLOW_DIRS = {
    "engine", "runtime", "modules", "web", "scripts/ci",
}

# Specific files in repo root the agent may modify/create
ALLOW_FILES = {
    "CMakeLists.txt", "README.md", "README_AUTOMATION.md",
}

# Paths the agent must NEVER touch (deny wins over allow)
DENY_PREFIXES = {
    ".github/",        # workflows, CODEOWNERS, etc.
    "scripts/agent/",  # this agent itself
}

def allowed_edit(path: str) -> bool:
    """
    Returns True if 'path' is permitted to be created/edited/deleted by the agent.
    """
    s = str(PurePosixPath(path)).replace("\\", "/")
    # deny always wins
    for dp in DENY_PREFIXES:
        if s == dp[:-1] or s.startswith(dp):
            return False
    # allow specific root files
    if s in ALLOW_FILES:
        return True
    # allow files under approved top-level dirs
    root = s.split("/", 1)[0]
    return root in ALLOW_DIRS

def ensure_codeowners(repo_root: Path) -> None:
    """
    Create .github/CODEOWNERS if missing (controlled exception).
    Assign ownership of critical paths to the repo owner so PRs require review.
    """
    gh_dir = repo_root / ".github"
    codeowners = gh_dir / "CODEOWNERS"
    if codeowners.exists():
        return
    # Default owner = org/user from GITHUB_REPOSITORY, fallback to your username
    owner = (os.environ.get("GITHUB_REPOSITORY", "/").split("/", 1)[0] or "WhiteHouseOffice").strip()
    gh_dir.mkdir(parents=True, exist_ok=True)
    content = f""".github/** @{owner}
scripts/agent/** @{owner}
"""
    codeowners.write_text(content, encoding="utf-8")
    print(f"Created {codeowners} to protect critical paths.")

def write_github_env(name: str, value: str) -> None:
    """
    Persist an env var to later workflow steps via $GITHUB_ENV.
    """
    path = os.environ.get("GITHUB_ENV")
    if path:
        with open(path, "a", encoding="utf-8") as f:
            f.write(f"{name}={value}\n")

# -------------- Main --------------

def main() -> None:
    repo_root = Path(".").resolve()

    # Read instruction text from stdin (provided by the workflow)
    instruction = sys.stdin.read()
    if not instruction.strip():
        print("No instruction text received on stdin. Exiting without error.")
        return

    SYSTEM = textwrap.dedent("""
    You are a careful repo-editing agent for a C/C++ game engine targeting Web (WebGPU via Emscripten) and native.
    Output ONLY a JSON object with this schema:
    {
      "commit_message": "string",
      "branch_suffix": "short-string-identifier",
      "files": [
        {"path": "relative/path", "content": "full UTF-8 file content"}
      ],
      "delete": ["optional/path/to/delete", "..."]
    }
    Rules:
    - Make minimal necessary changes to satisfy the user's instruction.
    - Always produce buildable code. If you add deps, update CMake/README.
    - Overwrite files entirely when you change them (no patch hunks).
    - No placeholders; working stubs with TODOs in comments are fine.
    - Keep YAML valid for any CI/workflow changes.
    """)

    model = os.environ.get("REPO_AGENT_MODEL", "gpt-4o")
    api_key = os.environ.get("OPENAI_API_KEY", "").strip()
    if not api_key:
        print("OPENAI_API_KEY not set; exiting gracefully.")
        return

    client = OpenAI(api_key=api_key)

    # Call the model
    resp = client.chat.completions.create(
        model=model,
        messages=[
            {"role": "system", "content": SYSTEM},
            {"role": "user", "content": instruction},
        ],
        temperature=0.2,
    )
    payload_txt = resp.choices[0].message.content or ""
    try:
        payload = json.loads(payload_txt)
    except json.JSONDecodeError as e:
        print("Model did not return valid JSON. First 1k chars:")
        print(payload_txt[:1000])
        raise

    # ---------------- Apply deletions (guarded) ----------------
    for pth in payload.get("delete", []) or []:
        if not isinstance(pth, str) or not pth.strip():
            continue
        if not allowed_edit(pth):
            print(f"SKIP DELETE (guardrails): {pth}")
            continue
        target = repo_root / pth
        if not target.exists():
            print(f"DELETE: {pth} (already missing)")
            continue
        if target.is_file():
            target.unlink()
            print(f"DELETE FILE: {pth}")
        else:
            import shutil
            shutil.rmtree(target, ignore_errors=True)
            print(f"DELETE DIR: {pth}")

    # ---------------- Apply file writes (guarded) ----------------
    for f in payload.get("files", []) or []:
        if not isinstance(f, dict):
            continue
        rel = f.get("path", "")
        content = f.get("content", "")
        if not rel:
            continue
        if not allowed_edit(rel):
            print(f"SKIP WRITE (guardrails): {rel}")
            continue
        p = repo_root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(content, encoding="utf-8")
        print(f"WRITE: {rel} ({len(content)} bytes)")

    # Ensure CODEOWNERS exists (controlled exception inside .github/)
    ensure_codeowners(repo_root)

    # ---------------- Commit on a new branch ----------------
    # Create a unique branch name
    run_id = os.environ.get("GITHUB_RUN_ID") or datetime.utcnow().strftime("%Y%m%d%H%M%S")
    suffix = payload.get("branch_suffix") or "changes"
    branch = f"bot/{suffix}-{run_id}"

    # Prepare commit message
    commit_msg = payload.get("commit_message") or "Automated changes from Repo Agent"

    # Git config & commit
    def run(*args):
        subprocess.check_call(list(args), cwd=str(repo_root))

    run("git", "config", "user.name", "repo-agent[bot]")
    run("git", "config", "user.email", "repo-agent@users.noreply.github.com")

    # Create branch from current HEAD
    run("git", "checkout", "-b", branch)
    run("git", "add", "-A")

    # Only commit if there are staged changes
    result = subprocess.run(["git", "diff", "--cached", "--quiet"], cwd=str(repo_root))
    if result.returncode != 0:
        run("git", "commit", "-m", commit_msg)
        run("git", "push", "-u", "origin", branch)
        print(f"PUSHED BRANCH: {branch}")
    else:
        print("No changes to commit; exiting gracefully.")
        # Still set env vars so the PR step doesn't choke
        write_github_env("REPO_AGENT_BRANCH", branch)
        write_github_env("REPO_AGENT_COMMIT_MESSAGE", commit_msg)
        return

    # Expose variables for the next workflow step (Create Pull Request)
    write_github_env("REPO_AGENT_BRANCH", branch)
    write_github_env("REPO_AGENT_COMMIT_MESSAGE", commit_msg)


if __name__ == "__main__":
    main()
