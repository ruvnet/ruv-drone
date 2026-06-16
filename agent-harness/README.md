# ruvdrone — agent harness for [ruv-drone](https://github.com/ruvnet/ruv-drone)

A repo-aware AI agent harness for **ruv-drone** (cooperative civilian UAV fleet
coordination in Rust), minted with [MetaHarness](https://github.com/ruvnet/agent-harness-generator).
It's a **flagship example**: one harness wired for **all 9 supported hosts**, and every host
config was **verified against the real host runtime** (not just schema-checked).

> **Scope:** civilian / industrial only (SAR, inspection, agriculture, mapping, telecom
> relay). The harness instructs agents to refuse weapons/targeting/engagement work — see
> ruv-drone's [`NOTICE`](../NOTICE) and `CLAUDE.md`.

## What you get

A small, repo-tuned agent: 4 agents (architect · implementer · reviewer · test-writer), a
`plan-change` skill, `doctor` + `review-diff` commands, a default-deny MCP policy, and a
memory namespace — all aware of ruv-drone's modules (`formation`, `allocation`, `planning`,
`marl`, `failsafe`, …) and Rust workflow (`cargo build/test/clippy/bench`).

## Use it on your host

The same harness drops into any of these — pick yours:

| Host | Config it reads | Quick start |
|------|-----------------|-------------|
| **Claude Code** | `.claude/settings.json`, `CLAUDE.md` | open the folder; `claude` |
| **OpenAI Codex** | `.codex/config.toml`, `AGENTS.md` | `codex` |
| **OpenCode** | `.opencode/opencode.json` | `opencode` |
| **GitHub Copilot** | `.vscode/mcp.json`, `.github/copilot-instructions.md` | trust workspace in VSCode 1.99+ |
| **GitHub Actions** | `.github/workflows/ruvdrone.yml` | commit + add a model key secret |
| **Hermes** | `cli-config.yaml` | `hermes` |
| **OpenClaw** | `.openclaw/openclaw.json` | `openclaw agent` |
| **pi.dev** | `AGENTS.md`, `SYSTEM.md`, `trust.json` | `pi` |
| **RVM** | `rvm.manifest.toml`, `capability-table.json` | `rvm-loader` |

Per-host install notes are in [`docs/hosts/`](./docs/hosts/).

## How it was built & verified

```bash
# minted with the MetaHarness CLI — one command, all 9 hosts (metaharness ≥ 0.1.14):
npx metaharness@latest ruvdrone \
  --host claude-code --host codex,copilot,github-actions,hermes,openclaw,opencode,pi-dev,rvm \
  --template vertical:coding
```

Each host's emitted config was loaded by the **actual runtime** and routed through a real
model (OpenRouter) to confirm it works end-to-end — e.g. `opencode run`, `codex doctor`,
`pi -p`, `hermes config show`, `openclaw config validate`, `act` for the workflow. (`rvm` is
bare-metal AArch64 and `copilot` is interactive VSCode, so those two are schema-verified.)

## Build

```bash
npm install
npm test
node bin/cli.js doctor
```

Built on [`@metaharness/kernel`](https://www.npmjs.com/package/@metaharness/kernel) — a
Rust→WASM/NAPI kernel that runs identically on every platform.

---

*The model is replaceable. The harness is the product.*
