# Project guidance for Claude

Limelight vision for an FTC team: ball cluster detection and ranking. See
`README.md` for how the pieces fit together.

## Hard rules

- Never use emoji anywhere in this project: not in source, comments,
  documentation, commit messages, pull request or issue text, or in chat
  replies about this project. Plain text only.

## Working style

- Keep the pure (numpy-only) logic in `limelight/ball_cluster_pipeline.py`
  testable off the Limelight; add or update tests in `tests/` for any logic
  change.
- All tunable parameters live in the `Config` block at the top of the pipeline.
