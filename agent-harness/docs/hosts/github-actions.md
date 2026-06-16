# Installing ruvdrone as a GitHub Actions harness

1. Commit `.github/workflows/ruvdrone.yml` + `.github/actions/ruvdrone/action.yml`.
2. Add your model-provider key as a repo secret — one of `ANTHROPIC_API_KEY`, `OPENROUTER_API_KEY`, or `OPENAI_API_KEY`.
3. Trigger: Actions → ruvdrone → Run workflow, or comment on an issue.
