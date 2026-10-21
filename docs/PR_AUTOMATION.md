# CDE1302 PR analysis and changelog

## Activation status

Configuration activated on main on 22 October 2026 through setup PR #1. The GEMINI_API_KEY secret is present and Actions is enabled; no API key is stored in this checkout. A documentation PR is testing live Gemini output and changelog writes. Successful API use and billing/quota state are not yet established.

The workflow adapts Harshiet-Nanda/CDE2310_G10_2526's PR-Agent setup. It keeps AI descriptions, reviews, inline improvement suggestions, automatic changelog commits and the same version counter rules. It does not use Release Please or create tags/releases. The 0.1.0 entry seeds this repository's own history, not the unrelated CDE2310 counter.

## Files

- `.github/workflows/pr_agent.yml`: GitHub events, pinned action revisions, Python 3.12, pinned `pr-agent[github]==0.48.0` and permissions.
- `.github/scripts/pr_agent_runner.py`: event/command authorization, commit-log section and one execution of each requested analysis command.
- `.github/scripts/test_pr_agent_runner.py`: offline authorization, configuration and commit-log checks.
- `.pr_agent.toml`: Gemini selection, exclusions and course-specific instructions.
- `CHANGELOG.md`: initial 0.1.0 baseline and future AI-generated entries.

## Operation

Ready, same-repository PRs targeting main trigger `describe`, `review`, `improve` and `update_changelog` once each when opened, reopened, marked ready or updated. Draft/fork/bot events are skipped. Per-PR execution is serialized; superseded push events are skipped before installation/model calls. Human PR text is passed as data through the GitHub API, not inserted into shell commands.

The author's PR description is preserved with a bounded, replaceable commit-log section. The AI description is posted as a comment rather than rewriting the author's title/body. Inline suggestions are advisory; the workflow does not apply code changes, approve or merge PRs, create PRs, flash hardware or run flight scripts.

Owners, organization members and repository collaborators may post one exact command on an eligible PR:

- `/describe`
- `/review`
- `/improve`
- `/update_changelog`

Other comments, edited comments, arguments and commands from other authors are not executed. No second automatic PR-Agent action duplicates the CLI calls.

The runner/configuration are checked out from the trusted base revision for PR events, or the default branch for comment events. PR source code is never executed. Remote PR/global settings loading is disabled before the CLI starts; the trusted TOML is loaded directly into the agent settings in an isolated child process. Changing automation/configuration in a PR therefore takes effect after it is merged into the base, not while reviewing that PR. This is why a separate test PR is needed after activation.

## Counter rules retained

AI reads the highest existing X.Y.Z version and makes one bump for its generated entry:

- `feat` AND exact word `MAJOR` in commit/PR -> X+1.0.0.
- `feat` without `MAJOR` -> X.Y+1.0.
- `fix`, `docs` or `test` without qualifying feat -> X.Y.Z+1.

Other maintenance/refactoring work is classified by its effect into those four categories. These are the original custom rules, not strict Conventional Commits/SemVer breaking-change behavior. The AI still chooses the number; inspect it before merging.

Like the original, `update_changelog` prepends to the PR branch's current changelog. Repeated updates/manual reruns can append another version even before merge. Different open PRs can choose the same next version; refresh/review the changelog against main before merging. Do not describe this as deterministic release automation or one-version-per-merge enforcement. Automatic commits use `[skip ci]` and GitHub's workflow token; bot events are excluded to avoid loops.

## Course-specific review focus

The goal is autonomous takeoff -> bounded AprilTag search -> local-frame localization -> return -> landing, with timestamped records and verified PX4 .ulg reconstruction for the report. Guidance targets ESP32/PX4/MAVLink/UDP, camera/body/NED transforms, image/pose synchronization, calibration assumptions, setpoint ownership and resource lifetimes. Relative body-frame detections do not by themselves meet local-coordinate reporting requirements.

Inherited swarm/relay examples are reference code, not assessed course functionality. ROS 2/shooter/docking assumptions have been removed from the adapted instructions. Binary materials/CAD, generated trajectories and vendored components are excluded from routine textual analysis. Changed binary content requires a factual author description; AI cannot validate geometry or flight outcomes from an unreadable diff. Vendored code changed deliberately needs focused manual review.

## Activation and cost

1. Add repository Actions secret `GEMINI_API_KEY` (never put it in a tracked file).
2. Permit Actions and existing-PR/content writes. Leave "Allow GitHub Actions to create and approve pull requests" unticked; this workflow needs neither action.
3. Initial setup has been merged into main (PR #1). Later automation changes use the same review/merge process.
4. Open a small documentation PR from a branch in this repository; inspect Actions logs, description/review comments and changelog commit.

Model: `gemini/gemini-3.8-flash`, including `model_weak`. No fallback model/provider. Input context capped at 32,000 tokens; this is not a monetary spending cap. Free-tier quotas, project billing and Actions minutes are separate concerns. Each PR update requests four analysis tools and can involve multiple model requests/retries. Review provider usage; avoid unnecessary manual reruns. No live model call or cost validation has been performed.

Free-tier model quota errors need not imply invalid credentials. Do not enable billing or switch providers automatically. Troubleshoot missing secret/authentication, 429 quota errors and GitHub write permissions separately. The CLI is configured to propagate tool failures so Actions can report them; analysis commands continue independently before the runner reports any failures.

## Validation

Offline unit tests (Python 3.12):

```sh
python -m unittest discover -s .github/scripts -p 'test_*.py' -v
```

Workflow syntax/expressions can be checked with `actionlint`. These checks make no GitHub writes, model calls or hardware connections. Live compatibility, secret validity, quotas, posting and changelog commits still require the activation test above.

`AGENTS.md` is currently ignored local guidance by user repository configuration. Maintain it locally after changes, but do not silently remove that ignore rule or claim GitHub automation can read it. Shared course context also lives in `README.md`, `docs/PROJECT_CONTEXT.md` and the explicit review instructions.

Initial live test revealed that PR-Agent needs its GitHub optional dependency and early failure propagation configured before request processing. The adapter/propagation fix is included in this PR; a fresh PR after merge will verify the trusted base configuration.

Verification checklist: confirm an AI description/review comment appears, inspect provider errors even if the workflow is green, and confirm the bot commit advances CHANGELOG.md from 0.1.0 before claiming end-to-end success.

The pinned release rejects restricted CLI settings such as fallback_models. Trusted TOML is now loaded through the Python settings interface, preserving exact types and avoiding those CLI restrictions. Live verification follows after merge.

Review evidence should link to the successful workflow run and generated changelog commit, so future maintainers can distinguish offline configuration validation from live integration validation.

Google returned 404 for Gemini 2.5 Flash: it is unavailable to new users and recommends Gemini 3.8 Flash. Both model slots now use Gemini 3.8 Flash (free tier listed by Google on 22 October 2026); no billing settings or counter rules changed.
