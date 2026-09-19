#!/usr/bin/env bash
#
# Personal, per-developer VS Code extensions for this dev container.
#
#     .devcontainer/install-personal-extensions.sh --add <id>...   install and remember
#     .devcontainer/install-personal-extensions.sh                 reinstall remembered
#     .devcontainer/install-personal-extensions.sh --help
#
# Extensions the whole team needs go in devcontainer.json under
# customizations.vscode.extensions. This is for the ones only you want.
#
# A rebuild wipes ~/.vscode-server/extensions, so anything you installed by hand
# is gone afterwards. The IDs you --add are remembered in personal-extensions.txt
# (gitignored, and on the /workspaces volume so it outlives the container), and
# the no-argument form reinstalls them. devcontainer.json runs that form as
# "postAttachCommand", so it happens on its own every time VS Code attaches.
#
# Without personal-extensions.txt this script does nothing, so it is a no-op for
# everyone who has not made a list of their own.

# Deliberately no -e: a bad extension ID must never block the attach.
set -uo pipefail

LIST="$(dirname "$0")/personal-extensions.txt"

usage() {
    cat <<'USAGE'
Personal, per-developer VS Code extensions for this dev container.

    .devcontainer/install-personal-extensions.sh --add <id>...   install and remember
    .devcontainer/install-personal-extensions.sh                 reinstall remembered
    .devcontainer/install-personal-extensions.sh --help

An extension's ID is on its Marketplace page, or in the Extensions view:
right-click the extension -> Copy Extension ID.
USAGE
}

require_code() {
    if ! command -v code >/dev/null 2>&1; then
        echo "personal extensions: 'code' CLI not on PATH" >&2
        return 1
    fi
}

# The IDs in the list file, one per line, minus comments and blank lines. The
# `|| [ -n "$line" ]` picks up a last line with no trailing newline, and the tr
# strips CR in case the file was ever written from a Windows editor.
list_ids() {
    [ -f "$LIST" ] || return 0
    while IFS= read -r line || [ -n "$line" ]; do
        id="${line%%#*}"
        id="$(printf '%s' "$id" | tr -d '[:space:]')"
        [ -n "$id" ] && printf '%s\n' "$id"
    done < "$LIST"
}

new_list() {
    cat > "$LIST" <<'HEADER'
# Your personal extensions, one ID per line. This file is gitignored: it is
# yours alone and nobody else gets what you list here. Extensions the whole
# team needs belong in devcontainer.json instead.
#
# Reinstalled automatically every time VS Code attaches to the container, so
# they survive "Dev Containers: Rebuild Container".

HEADER
}

remember() {
    if list_ids | grep -Fxqi -- "$1"; then
        echo "personal extensions: $1 is already in the list"
        return
    fi
    if [ ! -f "$LIST" ]; then
        new_list
    elif [ -n "$(tail -c 1 "$LIST")" ]; then
        printf '\n' >> "$LIST"  # previous line had no newline of its own
    fi
    printf '%s\n' "$1" >> "$LIST"
    echo "personal extensions: added $1"
}

cmd_add() {
    if [ "$#" -eq 0 ]; then
        echo "personal extensions: --add needs at least one extension ID" >&2
        usage >&2
        exit 2
    fi
    require_code || exit 1

    for id in "$@"; do
        if ! code --list-extensions 2>/dev/null | grep -Fxqi -- "$id"; then
            echo "personal extensions: installing $id"
            code --install-extension "$id" --force
            # Verify rather than trust the exit status: the CLI reports an
            # unknown ID on stdout and still exits 0.
            if ! code --list-extensions 2>/dev/null | grep -Fxqi -- "$id"; then
                echo "personal extensions: could not install $id, not adding it" >&2
                continue
            fi
        fi
        remember "$id"
    done
}

cmd_sync() {
    [ -f "$LIST" ] || exit 0
    require_code || exit 0  # nothing to do yet; never fail the attach

    # One snapshot up front, so an attach with nothing to do stays offline.
    installed="$(code --list-extensions 2>/dev/null)"

    list_ids | while IFS= read -r id; do
        printf '%s\n' "$installed" | grep -Fxqi -- "$id" && continue
        echo "personal extensions: installing $id"
        code --install-extension "$id" --force
    done
}

case "${1:-}" in
    --add) shift; cmd_add "$@" ;;
    -h|--help) usage ;;
    "") cmd_sync ;;
    *) echo "personal extensions: unknown option '$1'" >&2; usage >&2; exit 2 ;;
esac
