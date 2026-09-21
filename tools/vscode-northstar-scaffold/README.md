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

Then run **Developer: Reload Window**. You should not normally need to run this
by hand: `devcontainer.json` invokes it from `postCreateCommand` (via
`scripts/postcreate.sh`) and again, quietly, from `postAttachCommand`. Both,
because **Dev Containers: Rebuild Container** wipes `~/.vscode-server/extensions`
— the same reason `.devcontainer/install-personal-extensions.sh` re-syncs on
attach. A running window still needs the reload once.

## Why it's plain JavaScript

There is no Node toolchain in the devcontainer. VS Code runs extensions on its
own bundled Node and injects `require('vscode')`, so an unpacked, unbuilt
extension works — as long as it is *folder-scanned*, which means:

**The install directory must be named `<publisher>.<name>-<version>`** — so
`northstar.scaffold-0.0.1` for the current manifest. Any other name is silently
ignored. `scripts/install-extension.sh` reads those three fields straight out of
`package.json` and removes links from older versions, so bumping `version` here
needs no change anywhere else.

## Limits

- Not available on vscode.dev / github.dev (no Node extension host). Use the
  **Terminal → Run Task → Scaffold - New ...** tasks there instead.
- Unsigned and folder-scanned, so it won't appear in Settings Sync and won't
  auto-update. It updates in place whenever the repo does, since it's a symlink.
- It is *not* a personal extension: don't add it to
  `.devcontainer/personal-extensions.txt`. That list is for marketplace IDs
  installed via the `code` CLI; this one is a local folder.

## Files

| File | What it does |
|---|---|
| `package.json` | manifest: two commands, `onStartupFinished` activation |
| `extension.js` | status bar items + the QuickPick wizard; shells out to `scripts/scaffold` |
