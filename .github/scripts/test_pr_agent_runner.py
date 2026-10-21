"""Offline checks: no GitHub writes, model calls or vehicle connections."""
import copy
import importlib.util
from pathlib import Path
import unittest
import tomllib

spec = importlib.util.spec_from_file_location("runner", Path(__file__).with_name("pr_agent_runner.py"))
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def event():
    return {"action": "opened", "sender": {"type": "User"}, "pull_request": {
        "number": 3, "draft": False, "head": {"repo": {"full_name": "owner/course"}},
        "base": {"ref": "main"}}}


def comment(body="/review", association="OWNER"):
    return {"action": "created", "sender": {"type": "User"},
            "issue": {"number": 3, "pull_request": {"url": "unused"}},
            "comment": {"body": body, "author_association": association}}


class RunnerTests(unittest.TestCase):
    def test_same_repo_pr_runs_four_commands_once(self):
        self.assertEqual(runner.select_commands(event(), "pull_request", "owner/course"),
                         (3, list(runner.COMMANDS)))

    def test_reject_fork_draft_wrong_base_and_bot(self):
        for field in ("fork", "draft", "base", "bot"):
            e = copy.deepcopy(event())
            if field == "fork": e["pull_request"]["head"]["repo"]["full_name"] = "other/fork"
            if field == "draft": e["pull_request"]["draft"] = True
            if field == "base": e["pull_request"]["base"]["ref"] = "other"
            if field == "bot": e["sender"]["type"] = "Bot"
            with self.subTest(field=field), self.assertRaises(ValueError):
                runner.select_commands(e, "pull_request", "owner/course")

    def test_exact_trusted_commands(self):
        for command in runner.COMMANDS:
            self.assertEqual(runner.select_commands(comment("/" + command), "issue_comment", "owner/course"),
                             (3, [command]))

    def test_reject_untrusted_or_injected_commands(self):
        for body, association in [("/review", "NONE"), ("/review --config.model=other", "OWNER"),
                                  ("/review\n/improve", "OWNER"), ("review", "OWNER"),
                                  ("/merge", "OWNER")]:
            with self.subTest(body=body), self.assertRaises(ValueError):
                runner.select_commands(comment(body, association), "issue_comment", "owner/course")

    def test_reject_issue_and_edited_comment(self):
        issue = comment(); issue["issue"].pop("pull_request")
        edited = comment(); edited["action"] = "edited"
        for e in (issue, edited):
            with self.assertRaises(ValueError):
                runner.select_commands(e, "issue_comment", "owner/course")

    def test_commit_log_preserves_author_text_and_is_repeatable(self):
        commits = [{"sha": "123456789", "commit": {"message": "fix(mission): return home\nDetails"}}]
        body = "Problem and validation evidence."
        first = runner.commit_log_body(body, commits)
        self.assertEqual(first, runner.commit_log_body(first, commits))
        after = first + "\nMore human evidence."
        result = runner.commit_log_body(after, commits)
        self.assertIn("More human evidence.", result)
        self.assertEqual(result.count(runner.START), 1)

    def test_trusted_config_loaded_without_cli_overrides(self):
        config = tomllib.loads((runner.ROOT / ".pr_agent.toml").read_text())
        class Settings:
            def __init__(self): self.values = {}
            def set(self, key, value): self.values[key] = value
        settings = Settings()
        runner.apply_config(config, settings)
        for section, values in config.items():
            for field, value in values.items():
                self.assertEqual(settings.values[f"{section}.{field}"], value)
        self.assertEqual(settings.values["config.model_weak"], settings.values["config.model"])
        self.assertEqual(settings.values["config.fallback_models"], [])
        self.assertFalse(settings.values["config.use_repo_settings_file"])



if __name__ == "__main__":
    unittest.main()
