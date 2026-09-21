#!/usr/bin/env bash
#
# Devcontainer postCreateCommand. Referenced by .devcontainer/devcontainer.json
# -- if you add a step here, that file does not need to change, but if you
# rename this script it does.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$ROOT/northstar-robomaster-project"
pipenv install --dev --python /usr/bin/python3.11

# Link the scaffold extension. On a cold container the editor server directory
# may not exist yet; the script handles that and always exits 0, and
# devcontainer.json also runs it from postAttachCommand, which catches the
# rebuild case where ~/.vscode-server/extensions was wiped.
bash "$ROOT/scripts/install-extension.sh"
