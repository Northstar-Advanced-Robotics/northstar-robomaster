#!/usr/bin/env bash
#
# Devcontainer postCreateCommand. Referenced by .devcontainer/devcontainer.json
# -- if you add a step here, that file does not need to change, but if you
# rename this script it does.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$ROOT/northstar-robomaster-project"
pipenv install --dev --python /usr/bin/python3.11

# Best effort: a missing extensions dir on a cold container must not fail the
# whole postCreate. Teammates can re-run scripts/install-extension.sh.
bash "$ROOT/scripts/install-extension.sh" || \
    echo "warning: could not install the scaffold extension; run scripts/install-extension.sh later" >&2
