#!/usr/bin/env bash
#
# Installs the NorthStar Scaffold VS Code extension by symlinking it into the
# editor's extensions directory.
#
# There is no npm in this container, so the extension is plain JavaScript with
# no build step. `code --install-extension` needs a packaged .vsix (which needs
# vsce, which needs npm), and devcontainer.json's `customizations.vscode.
# extensions` only accepts marketplace IDs -- so a symlink is the route that
# works with the tools we actually have.
#
# Run this by hand if the buttons are missing, then: Developer: Reload Window.
#
# Usage: bash scripts/install-extension.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE="$ROOT/tools/vscode-northstar-scaffold"

# The folder name MUST be <publisher>.<name>-<version> or the extension host
# silently ignores it. These three values come from the extension's package.json.
TARGET_NAME="northstar.scaffold-0.0.1"

if [ ! -f "$SOURCE/package.json" ]; then
    echo "error: $SOURCE/package.json not found" >&2
    exit 1
fi

installed=0

# Always target VS Code's dir; also pick up any other editor server already here.
for server in "$HOME/.vscode-server" "$HOME/.vscode-server-insiders" \
              "$HOME/.cursor-server" "$HOME/.windsurf-server" "$HOME/.vscode"; do
    # Only create the extensions dir for plain VS Code; for the others, only
    # link if that editor is actually installed.
    if [ "$server" != "$HOME/.vscode-server" ] && [ ! -d "$server" ]; then
        continue
    fi
    mkdir -p "$server/extensions"
    ln -sfn "$SOURCE" "$server/extensions/$TARGET_NAME"
    echo "linked $server/extensions/$TARGET_NAME"
    installed=$((installed + 1))
done

if [ "$installed" -eq 0 ]; then
    echo "warning: no editor server directory found; nothing linked" >&2
    exit 0
fi

echo
echo "Done. Run 'Developer: Reload Window' to pick it up."
echo "You should then see 'New Subsystem' and 'New Command' in the status bar."
