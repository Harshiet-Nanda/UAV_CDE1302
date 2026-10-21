"""Run only approved PR-Agent commands using trusted base-branch configuration."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import tomllib

COMMANDS = ("describe", "review", "improve", "update_changelog")
TRUSTED = {"OWNER", "MEMBER", "COLLABORATOR"}
ROOT = Path(__file__).resolve().parents[2]
START = "<!-- cde1302-commit-log:start -->"
END = "<!-- cde1302-commit-log:end -->"


def select_commands(event: dict, event_name: str, repository: str) -> tuple[int, list[str]]:
    """Pure event gate; a comment is a command only if the entire body matches."""
    if event.get("sender", {}).get("type") == "Bot":
        raise ValueError("Bot events are not processed")
    if event_name == "pull_request":
        pr = event["pull_request"]
        if event.get("action") not in {"opened", "reopened", "ready_for_review", "synchronize"}:
            raise ValueError("Unsupported PR event")
        if pr.get("draft") or pr["head"]["repo"]["full_name"] != repository or pr["base"]["ref"] != "main":
            raise ValueError("Only ready, same-repository PRs into main are supported")
        return pr["number"], list(COMMANDS)
    if event_name == "issue_comment":
        comment = event["comment"]
        command = comment["body"].removeprefix("/")
        if (event.get("action") != "created" or not event["issue"].get("pull_request")
                or comment.get("author_association") not in TRUSTED
                or comment["body"] != f"/{command}" or command not in COMMANDS):
            raise ValueError("Comment is not an authorized exact command")
        return event["issue"]["number"], [command]
    raise ValueError("Unsupported workflow event")


def gh_json(endpoint: str, *, paginate: bool = False):
    args = ["gh", "api", endpoint]
    if paginate:
        args += ["--paginate", "--slurp"]
    return json.loads(subprocess.check_output(args, text=True))


def commit_log_body(body: str, commits: list[dict]) -> str:
    """Refresh a bounded bot-owned section without truncating author text."""
    if START in body and END in body:
        before, rest = body.split(START, 1)
        _, after = rest.split(END, 1)
        body = before.rstrip() + after
    lines = [f"- `{c['sha'][:7]}` {c['commit']['message'].splitlines()[0]}" for c in commits]
    return body.rstrip() + "\n\n" + START + "\n### Auto-generated commit log\n" + "\n".join(lines) + "\n" + END + "\n"


def config_arguments(config: dict) -> list[str]:
    args = []
    for section, settings in config.items():
        for key, value in settings.items():
            # Pass data as argv, never interpolate PR text into a shell.
            encoded = json.dumps(value)
            args.append(f"--{section}.{key}={encoded}")
    return args


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    event = json.loads(Path(os.environ["GITHUB_EVENT_PATH"]).read_text())
    repository = os.environ["GITHUB_REPOSITORY"]
    number, commands = select_commands(event, os.environ["GITHUB_EVENT_NAME"], repository)
    pr = gh_json(f"repos/{repository}/pulls/{number}")
    if (pr["state"] != "open" or pr["draft"] or pr["head"]["repo"]["full_name"] != repository
            or pr["base"]["ref"] != "main"):
        raise ValueError("PR is no longer eligible")
    # Avoid spending calls on an outdated synchronize event queued behind a newer push.
    if os.environ["GITHUB_EVENT_NAME"] == "pull_request" and event["pull_request"]["head"]["sha"] != pr["head"]["sha"]:
        print("Superseded PR event; no model calls made.")
        if args.check:
            with open(os.environ["GITHUB_ENV"], "a") as stream:
                stream.write("PR_AGENT_SUPERSEDED=true\n")
        return
    if args.check:
        print(f"Authorized PR #{number}: {', '.join(commands)}")
        return
    if not os.environ.get("GEMINI_API_KEY"):
        raise ValueError("Add the GEMINI_API_KEY repository Actions secret before running")
    config = tomllib.loads((ROOT / ".pr_agent.toml").read_text())
    if config["config"]["use_repo_settings_file"] or config["config"]["use_global_settings_file"]:
        raise ValueError("Only checked-out trusted configuration may be used")
    if os.environ["GITHUB_EVENT_NAME"] == "pull_request":
        pages = gh_json(f"repos/{repository}/pulls/{number}/commits?per_page=100", paginate=True)
        commits = [commit for page in pages for commit in page]
        body = commit_log_body(pr.get("body") or "", commits)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "body.json"
            path.write_text(json.dumps({"body": body}))
            subprocess.run(["gh", "api", f"repos/{repository}/pulls/{number}",
                            "--method", "PATCH", "--input", str(path), "--silent"], check=True)
    # Repository settings load before CLI overrides, so disable remote loading
    # in the child environment as well as in trusted TOML.
    os.environ["config.use_repo_settings_file"] = "false"
    os.environ["config.use_global_settings_file"] = "false"
    os.environ["config.propagate_tool_errors"] = "true"
    overrides = config_arguments(config)
    failures = []
    for command in commands:
        print(f"Running {command} on PR #{number}", flush=True)
        result = subprocess.run([sys.executable, "-m", "pr_agent.cli", f"--pr_url={pr['html_url']}",
                                 command, *overrides], cwd=ROOT)
        if result.returncode:
            failures.append(command)
    if failures:
        raise RuntimeError("PR-Agent commands failed: " + ", ".join(failures))


if __name__ == "__main__":
    main()
