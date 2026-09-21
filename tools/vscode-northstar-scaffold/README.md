# NorthStar Scaffold

Two status-bar buttons — **New Subsystem** and **New Command** — that generate
boilerplate under `northstar-robomaster-project/src/control/` and print an exact
checklist for wiring the result into each robot's `*_control.cpp`.

All the real work happens in `scripts/scaffold` (Python). This extension is only
a wizard around it, so the buttons, the `Scaffold - New ...` tasks and the CLI
all behave identically.

## Install

```bash
bash scripts/install-extension.sh
```

Then run **Developer: Reload Window**. The devcontainer's `postCreateCommand`
does this for you on a fresh container, but a running window still needs the
reload once.

## Why it's plain JavaScript

There is no Node toolchain in the devcontainer. VS Code runs extensions on its
own bundled Node and injects `require('vscode')`, so an unpacked, unbuilt
extension works — as long as it is *folder-scanned*, which means:

**The install directory must be named `northstar.scaffold-0.0.1`** —
`<publisher>.<name>-<version>` from `package.json`. Any other name is silently
ignored. If you bump `version` in `package.json`, update `TARGET_NAME` in
`scripts/install-extension.sh` to match, or the old symlink keeps winning.

## Limits

- Not available on vscode.dev / github.dev (no Node extension host). Use the
  **Terminal → Run Task → Scaffold - New ...** tasks there instead.
- Unsigned and folder-scanned, so it won't appear in Settings Sync and won't
  auto-update. It updates in place whenever the repo does, since it's a symlink.

## Files

| File | What it does |
|---|---|
| `package.json` | manifest: two commands, `onStartupFinished` activation |
| `extension.js` | status bar items + the QuickPick wizard; shells out to `scripts/scaffold` |
