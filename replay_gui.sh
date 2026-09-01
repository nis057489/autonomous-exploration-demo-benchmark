#!/usr/bin/env bash
set -euo pipefail

# Launches replay_gui.py with whichever Python actually has tkinter.
# System python3 typically lacks it; Homebrew's python-tk formula (if
# installed) provides it but linuxbrew sits behind /usr/bin on PATH, so it
# has to be found explicitly. Falls back to running inside the jazzy_env
# distrobox (which has tkinter) if nothing on the host does -- there, the
# GUI shells out to the host via distrobox-host-exec to run
# replay_compare.sh, since docker isn't available inside the container.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

has_tkinter() {
  "$1" -c "import tkinter" >/dev/null 2>&1
}

for candidate in python3 /home/linuxbrew/.linuxbrew/bin/python3; do
  if command -v "${candidate}" >/dev/null 2>&1 && has_tkinter "${candidate}"; then
    exec "${candidate}" "${SCRIPT_DIR}/replay_gui.py" "$@"
  fi
done

echo "No host Python with tkinter found -- falling back to jazzy_env distrobox." >&2
exec distrobox enter jazzy_env -- python3 "${SCRIPT_DIR}/replay_gui.py" "$@"
