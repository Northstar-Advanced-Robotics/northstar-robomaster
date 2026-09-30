#!/usr/bin/env bash
#
# Regenerates every board's taproot/modm tree from the taproot submodule (LateralRoot).
# Run from anywhere in the repo:
#
#     ./scripts/regenerate_taproot.sh
#
# lbuild has to run from inside the folder holding the board's project.xml: Taproot builds its modm
# sub-project relative to the current directory, so `lbuild -p <elsewhere>` would reuse the wrong modm.
# Never hand-edit the generated trees; change LateralRoot and rerun this instead.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_DIR="$REPO_ROOT/northstar-robomaster-project"

# The board folders have no Pipfile of their own, so point pipenv at the project's.
export PIPENV_PIPFILE="$PROJECT_DIR/Pipfile"

for board_dir in "$PROJECT_DIR"/boards/*/; do
    board="$(basename "$board_dir")"
    echo "==> Generating $board"
    (cd "$board_dir" && pipenv run lbuild build)
done

echo "Done. Review the changes with: git status && git diff"
