#!/usr/bin/env bash
#
# Installs the NorthStar Scaffold VS Code extension (the "New Subsystem" /
# "New Command" status bar buttons) by symlinking tools/vscode-northstar-scaffold
# into the editor's extensions directory.
#
#     bash scripts/install-extension.sh            install, with output
#     bash scripts/install-extension.sh --quiet    only speak up if something changed
#     bash scripts/install-extension.sh --help
#
# There is no npm in this container, so the extension is plain JavaScript with
# no build step. `code --install-extension` needs a packaged .vsix (which needs
# vsce, which needs npm), and devcontainer.json's customizations.vscode.
# extensions only accepts marketplace IDs -- so a symlink is the route that
# works with the tools we actually have.
#
# devcontainer.json runs this from both "postCreateCommand" (via
# scripts/postcreate.sh) and "postAttachCommand". Both, because a rebuild wipes
# ~/.vscode-server/extensions -- the same reason
# .devcontainer/install-personal-extensions.sh re-syncs on attach.
#
# Deliberately no -e: a failure here must never block the attach.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE="$ROOT/tools/vscode-northstar-scaffold"
MANIFEST="$SOURCE/package.json"

QUIET=0
case "${1:-}" in
    --quiet|-q) QUIET=1 ;;
    -h|--help) sed -n '3,9p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    "") ;;
    *) echo "scaffold extension: unknown option '$1'" >&2; exit 2 ;;
esac

say() { [ "$QUIET" -eq 1 ] || echo "$@"; }
changed() { echo "$@"; }   # always worth printing, even on attach

if [ ! -f "$MANIFEST" ]; then
    echo "scaffold extension: $MANIFEST not found" >&2
    exit 0
fi

# The install directory MUST be <publisher>.<name>-<version> or the extension
# host silently ignores it. Read those from package.json rather than hardcoding
# them, so bumping the version here cannot leave a stale link winning.
NAME="$(python3 -c 'import json,sys
d=json.load(open(sys.argv[1]))
print("%s.%s-%s" % (d["publisher"], d["name"], d["version"]))' "$MANIFEST" 2>/dev/null)"

if [ -z "$NAME" ]; then
    echo "scaffold extension: could not read publisher/name/version from $MANIFEST" >&2
    exit 0
fi

linked_any=0
made_change=0

# Always target VS Code's directory; also pick up any other editor server that
# is actually present, so Cursor/Windsurf users get the buttons too.
for server in "$HOME/.vscode-server" "$HOME/.vscode-server-insiders" \
              "$HOME/.cursor-server" "$HOME/.windsurf-server" "$HOME/.vscode"; do
    if [ "$server" != "$HOME/.vscode-server" ] && [ ! -d "$server" ]; then
        continue
    fi

    ext_dir="$server/extensions"
    link="$ext_dir/$NAME"

    if ! mkdir -p "$ext_dir" 2>/dev/null; then
        echo "scaffold extension: cannot create $ext_dir" >&2
        continue
    fi

    # Drop links from an older version of this extension, otherwise VS Code
    # loads both and the stale one may win.
    for stale in "$ext_dir"/northstar.scaffold-*; do
        [ -L "$stale" ] || continue
        [ "$stale" = "$link" ] && continue
        if [ "$(readlink "$stale")" = "$SOURCE" ]; then
            rm -f "$stale"
            changed "scaffold extension: removed stale $(basename "$stale")"
            made_change=1
        fi
    done

    linked_any=1

    if [ -L "$link" ] && [ "$(readlink "$link")" = "$SOURCE" ]; then
        say "scaffold extension: already linked at $link"
        continue
    fi

    if ln -sfn "$SOURCE" "$link" 2>/dev/null; then
        changed "scaffold extension: linked $link"
        made_change=1
    else
        echo "scaffold extension: could not link $link" >&2
    fi
done

if [ "$linked_any" -eq 0 ]; then
    say "scaffold extension: no editor server directory found; nothing to do"
    exit 0
fi

if [ "$made_change" -eq 1 ] || [ "$QUIET" -eq 0 ]; then
    echo
    echo "Run 'Developer: Reload Window' to pick it up. You should then see"
    echo "'New Subsystem' and 'New Command' in the status bar."
fi

exit 0
