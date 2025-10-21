# scripts/agent/repo_agent.py
import os, sys, json, subprocess, textwrap
from pathlib import Path

try:
    from openai import OpenAI
except Exception as e:
    print("ERROR: openai package not available", e, file=sys.stderr)
    sys.exit(1)

def main():
    repo_root = Path(".")
    instruction = sys.stdin.read()
    if not instruction.strip():
        print("No instruction text received on stdin.", file=sys.stderr)
        sys.exit(2)

    SYSTEM = textwrap.dedent('''
    You are a careful repo-editing agent for a C/C++ game engine project targeting Web (WebGPU) and native.
    Output only a JSON object with this schema:
    {
      "commit_message": "string",
      "branch_suffix": "short-string-identifier",
      "files": [
        {"path": "relative/path.ext", "content": "full file content (UTF-8)"}
      ],
      "delete": ["optional/path/to/delete", "..."]
    }
    Rules:
    - Make minimal necessary changes to satisfy the user's instruction.
    - Always produce buildable code. If you add dependencies, update CMake/README as needed.
    - Overwrite files entirely when you change them.
    - Do not output placeholders; prefer working stubs and TODOs as comments.
    - Keep YAML valid for any CI workflow changes.
    ''')

    model = os.environ.get("REPO_AGENT_MODEL", "gpt-4o")
    client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])

    resp = client.chat.completions.create(
        model=model,
        messages=[{"role":"system","content":SYSTEM},{"role":"user","content":instruction}],
        temperature=0.2,
    )

    payload_txt = resp.choices[0].message.content
    try:
        payload = json.loads(payload_txt)
    except json.JSONDecodeError as e:
        print("Model did not return valid JSON:", e, file=sys.stderr)
        print(payload_txt[:1000], file=sys.stderr)
        sys.exit(3)

    # Apply deletions
    for p in payload.get("delete", []):
        path = repo_root / p
        if path.exists():
            if path.is_file():
                path.unlink()
            else:
                # optional: delete dirs recursively
                import shutil
                shutil.rmtree(path, ignore_errors=True)

    # Apply file writes
    for f in payload.get("files", []):
        p = repo_root / f["path"]
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(f["content"], encoding="utf-8")

    # Commit on a new branch so the workflow can open a PR
    branch_suffix = payload.get("branch_suffix", "changes")
    branch = f"bot/{branch_suffix}"
    os.environ["REPO_AGENT_BRANCH"] = branch
    commit_msg = payload.get("commit_message", "Automated changes from Repo Agent")
    os.environ["REPO_AGENT_COMMIT_MESSAGE"] = commit_msg

    subprocess.check_call(["git","config","user.name","repo-agent[bot]"])
    subprocess.check_call(["git","config","user.email","repo-agent@users.noreply.github.com"])
    subprocess.check_call(["git","checkout","-b", branch])
    subprocess.check_call(["git","add","-A"])
    # Only commit if there are changes
    result = subprocess.run(["git","diff","--cached","--quiet"])
    if result.returncode != 0:
        subprocess.check_call(["git","commit","-m", commit_msg])
    else:
        print("No changes to commit; exiting gracefully.")

if __name__ == "__main__":
    main()
