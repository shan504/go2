#!/usr/bin/env bash
# Build the pinned optional backend in a separate venv. No sudo, ROS startup,
# motion commands, modification of the stable environment, or service restart.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
python=${KISS_PYTHON:-python3}
venv=${KISS_VENV:-$root/.venv-kiss}
dependencies=${KISS_DEPS:-$root/.kiss-deps}
commit=db81ee31b59bc7a18af3fec8641d5867646ab57a

if [[ ${1:-} == --help ]]; then
  echo 'Usage: KISS_PYTHON=python3.10 bash go2_3d/setup_kiss_env.sh'
  echo 'Optional KISS_VENV/KISS_DEPS select NEW isolated locations (default: this worktree).'
  echo 'Requires C/C++ compiler, make, git, patch and Python development headers.'
  exit 0
fi
[[ $# == 0 ]] || { echo 'Unexpected arguments; use --help' >&2; exit 2; }
for tool in git patch "${KISS_CC:-gcc}" "${KISS_CXX:-g++}"; do
  command -v "$tool" >/dev/null || { echo "Missing build tool: $tool" >&2; exit 2; }
done
"$python" - <<'PY'
import platform, sys, sysconfig
from pathlib import Path
if platform.machine() not in ('x86_64', 'AMD64') or sys.platform != 'linux':
    raise SystemExit('This locked bootstrap currently supports Linux x86_64. Jetson ARM64 requires a separately built Open3D 0.19 wheel; see docs/KISS_DEPENDENCIES.md. No environment was changed.')
if sys.version_info[:2] not in ((3, 10), (3, 11), (3, 12)):
    raise SystemExit('Use Python 3.10 for Humble, or Python 3.10–3.12 for offline evaluation')
if not (Path(sysconfig.get_path('include')) / 'Python.h').is_file():
    raise SystemExit('Python development headers missing; install in a dedicated build image')
PY
if [[ -e "$venv" ]]; then
  [[ -f "$venv/.go2-kiss-owned" ]] || {
    echo "Refusing to change an existing environment without our ownership marker: $venv" >&2
    exit 2
  }
else
  "$python" -m venv "$venv"
  touch "$venv/.go2-kiss-owned"
fi
"$venv/bin/python" - "$python" <<'PY'
import subprocess, sys
wanted = subprocess.check_output([sys.argv[1], '-c', 'import sys; print(sys.version_info[:2])'], text=True).strip()
if wanted != str(sys.version_info[:2]):
    raise SystemExit('Existing KISS venv uses a different Python ABI; choose a new KISS_VENV')
PY
mkdir -p "$dependencies"
source_dir=$dependencies/kiss-slam
if [[ ! -e "$source_dir" ]]; then
  git clone --no-checkout https://github.com/PRBonn/kiss-slam.git "$source_dir"
  git -C "$source_dir" checkout --detach "$commit"
fi
[[ -d "$source_dir/.git" ]] || { echo 'KISS dependency directory is not a Git checkout' >&2; exit 2; }
[[ -z $(git -C "$source_dir" status --porcelain --untracked-files=no) ]] || {
  echo 'Refusing to overwrite modified KISS-SLAM source' >&2; exit 2;
}
git -C "$source_dir" cat-file -e "$commit^{commit}" 2>/dev/null || git -C "$source_dir" fetch origin "$commit"
git -C "$source_dir" checkout --detach "$commit"
"$venv/bin/python" -m pip install --require-hashes -r "$root/requirements-kiss.lock"
# Use only this venv's CMake/Ninja. Native libraries use upstream's pinned
# vendored versions, not arbitrary existing system g2o/Eigen installations.
PATH="$venv/bin:$PATH" CC="${KISS_CC:-gcc}" CXX="${KISS_CXX:-g++}" \
  CMAKE_BUILD_PARALLEL_LEVEL="${KISS_BUILD_JOBS:-2}" \
  "$venv/bin/python" -m pip install --no-deps --no-build-isolation \
    --config-settings=cmake.define.USE_SYSTEM_EIGEN3=OFF \
    --config-settings=cmake.define.USE_SYSTEM_G2O=OFF \
    --config-settings=cmake.define.USE_SYSTEM_TSL-ROBIN-MAP=OFF \
    --config-settings=cmake.define.CMAKE_EXPORT_PACKAGE_REGISTRY=OFF \
    --config-settings=cmake.define.CMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON \
    --config-settings=cmake.define.BUILD_TESTING=OFF \
    --config-settings=cmake.define.CMAKE_CXX_FLAGS=-Wno-error=maybe-uninitialized "$source_dir"
"$venv/bin/python" "$root/go2_3d/check_kiss_env.py" --record-source "$source_dir" --smoke
"$venv/bin/python" -m pip check
printf 'KISS environment ready: %s\n' "$venv"
