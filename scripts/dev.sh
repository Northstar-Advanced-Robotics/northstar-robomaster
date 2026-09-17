#!/usr/bin/env bash
#
# CLI entry point for the dev container. Builds the image if needed, then drops
# you into a shell inside it. Run from anywhere in the repo:
#
#     ./scripts/dev.sh                  # interactive shell
#     ./scripts/dev.sh make -j8         # run one command and exit
#
# NOTE: container settings live in two places and must be kept in sync:
#   - the knobs below                       -> CLI workflow
#   - .devcontainer/devcontainer.json       -> VS Code workflow
# If you add a device, mount, or network flag to one, add it to the other.

set -euo pipefail

IMAGE="northstar-robomaster:dev"
DOCKERFILE=".devcontainer/Dockerfile"
# devcontainer.json has no "context", so VS Code builds with .devcontainer/ as
# the context. Match that here or the two workflows can produce different images.
BUILD_CONTEXT=".devcontainer"

# Mirrors "remoteUser" in devcontainer.json.
CONTAINER_USER="vscode"

# Mirrors the directory in "postCreateCommand".
PROJECT_DIR="northstar-robomaster-project"

# --- repo-specific flags ---------------------------------------------------
# Mirror any change here in devcontainer.json "runArgs" (currently absent, so
# this stays empty). Example if you ever pass through a J-Link:
#   RUN_FLAGS=( "--device" "/dev/bus/usb" )
RUN_FLAGS=()
# ---------------------------------------------------------------------------

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# Prefer docker, fall back to podman.
ENGINE="docker"
command -v docker >/dev/null 2>&1 || ENGINE="podman"

# SELinux (Fedora/RHEL) and podman need the :Z relabel on bind mounts.
MOUNT_OPT=""
if [ "$ENGINE" = "podman" ] || [ -d /sys/fs/selinux ]; then
  MOUNT_OPT=":Z"
fi

"$ENGINE" build -t "$IMAGE" -f "$DOCKERFILE" "$BUILD_CONTEXT"

# --rm means the container filesystem is thrown away every run, so the pipenv
# venv has to live in the bind-mounted repo to survive. PIPENV_VENV_IN_PROJECT
# puts it at $PROJECT_DIR/.venv, which is in .gitignore.
exec "$ENGINE" run -it --rm \
  -u "$CONTAINER_USER" \
  -v "${REPO_ROOT}:/ws${MOUNT_OPT}" \
  -w "/ws/${PROJECT_DIR}" \
  -e PIPENV_VENV_IN_PROJECT=1 \
  ${RUN_FLAGS[@]+"${RUN_FLAGS[@]}"} \
  "$IMAGE" bash -lc '
    if [ ! -d .venv ]; then
      pipenv install --dev --python /usr/bin/python3.11
    fi
    exec "$@"
  ' _ "${@:-bash}"
