#!/usr/bin/env bash
# Rebuild the WASM demo and push the built static site (NOT this repo's
# source) to the public wavestream_demo repo. See web/README.md.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_DIR="${SCRIPT_DIR}/web"
SITE_DIR="${WEB_DIR}/site"
PUBLIC_REPO_DIR="${PUBLIC_REPO_DIR:-${HOME}/code/wavestream_demo}"
PUBLIC_REMOTE="git@github.com:nis057489/wavestream_demo.git"

if [[ ! -d "${HOME}/emsdk" ]]; then
  echo "~/emsdk not found -- install the Emscripten SDK first:" >&2
  echo "  git clone https://github.com/emscripten-core/emsdk.git ~/emsdk" >&2
  echo "  ~/emsdk/emsdk install latest && ~/emsdk/emsdk activate latest" >&2
  exit 1
fi
# shellcheck source=/dev/null
source "${HOME}/emsdk/emsdk_env.sh" > /dev/null

echo "Building ${WEB_DIR}..."
emcmake cmake -S "${WEB_DIR}" -B "${WEB_DIR}/build" -DCMAKE_BUILD_TYPE=Release > /dev/null
cmake --build "${WEB_DIR}/build" -j

if [[ ! -d "${PUBLIC_REPO_DIR}/.git" ]]; then
  echo "Cloning ${PUBLIC_REMOTE} to ${PUBLIC_REPO_DIR}..."
  git clone "${PUBLIC_REMOTE}" "${PUBLIC_REPO_DIR}"
fi

echo "Copying built site to ${PUBLIC_REPO_DIR}..."
cp "${SITE_DIR}/index.html" "${SITE_DIR}/style.css" "${SITE_DIR}/app.js" \
   "${SITE_DIR}/vxch.js" "${SITE_DIR}/vxch.wasm" "${PUBLIC_REPO_DIR}/"

cd "${PUBLIC_REPO_DIR}"
git add index.html style.css app.js vxch.js vxch.wasm
if git diff --cached --quiet; then
  echo "No changes to publish."
  exit 0
fi

git commit -m "Update built demo"
echo "Committed. Review with 'git -C ${PUBLIC_REPO_DIR} show', then push:"
echo "  git -C ${PUBLIC_REPO_DIR} push"
