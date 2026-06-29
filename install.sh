#!/usr/bin/env bash
# meshflow installer — run from the repo root
set -euo pipefail

RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
BLD='\033[1m'
RST='\033[0m'

ok()   { echo -e "${GRN}  [✓]${RST} $*"; }
warn() { echo -e "${YLW}  [!]${RST} $*"; }
die()  { echo -e "${RED}  [✗]${RST} $*"; exit 1; }
step() { echo -e "\n${BLD}▶ $*${RST}"; }

echo ""
echo -e "${BLD}  meshflow installer${RST}"
echo "  ────────────────────────────────────────"

# ── 0. Must be run from the repo root ───────────────────────────────────────
[[ -f pyproject.toml ]] || die "Run install.sh from the meshflow directory."

# ── 1. Python ≥ 3.10 ─────────────────────────────────────────────────────────
step "Checking Python"
command -v python3 &>/dev/null || die "python3 not found. Install Python 3.10+ first."
PY_MINOR=$(python3 -c 'import sys; print(sys.version_info.minor)')
PY_MAJOR=$(python3 -c 'import sys; print(sys.version_info.major)')
[[ $PY_MAJOR -ge 3 && $PY_MINOR -ge 10 ]] \
    || die "Python 3.10+ required (found $PY_MAJOR.$PY_MINOR)."
ok "Python $(python3 --version)"

# ── 2. uv ────────────────────────────────────────────────────────────────────
step "Checking uv"
if ! command -v uv &>/dev/null; then
    warn "uv not found — installing now..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    # astral installer puts uv in ~/.local/bin
    export PATH="$HOME/.local/bin:$PATH"
    command -v uv &>/dev/null || die "uv installation failed. Install manually: https://docs.astral.sh/uv"
    ok "uv installed ($(uv --version))"
else
    ok "uv $(uv --version)"
fi

# ── 3. Python deps (onshape-to-robot, dotenv, numpy, trimesh …) ─────────────
step "Installing Python dependencies"
uv sync --quiet
ok "Dependencies ready"

# ── 4. Install meshflow as a global uv tool ──────────────────────────────────
step "Installing meshflow CLI"
# --editable so local source changes are picked up without re-running install
uv tool install --editable . --force --quiet
ok "meshflow CLI installed"

# ── 5. PATH check ────────────────────────────────────────────────────────────
step "Checking PATH"
UV_TOOL_BIN="$HOME/.local/bin"
PATH_OK=false
if command -v meshflow &>/dev/null; then
    ok "meshflow is on PATH  →  $(command -v meshflow)"
    PATH_OK=true
else
    warn "meshflow is not on PATH yet."
    warn "Add this line to your ~/.bashrc or ~/.zshrc:"
    echo  ""
    echo  "      export PATH=\"\$HOME/.local/bin:\$PATH\""
    echo  ""
    warn "Then reload your shell:  source ~/.zshrc   (or ~/.bashrc)"
fi

# ── 6. ROS 2 reminder ────────────────────────────────────────────────────────
step "ROS 2 check"
if [[ -n "${ROS_DISTRO:-}" ]]; then
    ok "ROS 2 $ROS_DISTRO detected"
else
    warn "ROS 2 not sourced (ROS_DISTRO not set)."
    warn "Before launching a generated package, source ROS 2:"
    warn "  source /opt/ros/<distro>/setup.bash"
    warn "Required ROS 2 packages (apt):"
    warn "  ros-\$ROS_DISTRO-gazebo-ros-pkgs"
    warn "  ros-\$ROS_DISTRO-robot-state-publisher"
    warn "  ros-\$ROS_DISTRO-joint-state-publisher-gui"
    warn "  ros-\$ROS_DISTRO-xacro"
fi

# ── Done ─────────────────────────────────────────────────────────────────────
echo ""
echo -e "${BLD}  ══════════════════════════════════════════${RST}"
if $PATH_OK; then
    echo -e "${BLD}  meshflow is ready. Run:${RST}"
    echo ""
    echo "    meshflow --help"
    echo "    meshflow init      ← set your Onshape API keys"
    echo "    meshflow           ← start converting"
else
    echo -e "${BLD}  After adding ~/.local/bin to PATH, run:${RST}"
    echo ""
    echo "    meshflow init      ← set your Onshape API keys"
fi
echo -e "${BLD}  ══════════════════════════════════════════${RST}"
echo ""
