#!/usr/bin/env bash
# Environment setup for TT_INet_MCU simulation, lint, formal, and SKY130 synthesis.
# Usage: source scripts/env.sh

if [ -f "$HOME/.local/oss-cad-suite/environment" ]; then
    # shellcheck disable=SC1091
    source "$HOME/.local/oss-cad-suite/environment"
fi

export SKY130_LIB="${SKY130_LIB:-$HOME/.local/sky130/sky130_fd_sc_hd__tt_025C_1v80.lib}"
export TT_VENV_PY="${TT_VENV_PY:-$HOME/.local/tt-sim-venv/bin/python}"

if [ ! -x "$TT_VENV_PY" ]; then
    export TT_VENV_PY="python3"
fi
