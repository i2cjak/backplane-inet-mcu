#!/usr/bin/env bash
# Synthesize TT_INet_MCU (or a specified module) against SKY130 HD liberty
# and report exact cell counts, DFF count, area (um^2), and 8x2 utilization.
#
# Usage:
#   ./scripts/synth_sky130.sh
#   ./scripts/synth_sky130.sh --top <module_name> [src/file1.v src/file2.v ...]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"

if [ ! -f "$SKY130_LIB" ]; then
    echo "ERROR: SKY130 liberty not found at $SKY130_LIB" >&2
    exit 1
fi

TOP_OVERRIDE=""
if [ "${1:-}" = "--top" ]; then
    if [ $# -lt 2 ]; then
        echo "Usage: $0 [--top <module_name> [files...]]" >&2
        exit 1
    fi
    TOP_OVERRIDE="$2"
    shift 2
fi

FILE_OVERRIDES=("$@")

# Extract top_module, tiles, and source_files from info.yaml
INFO_JSON="$("$TT_VENV_PY" - "$REPO_ROOT/info.yaml" <<'PYEOF'
import json, sys, yaml
with open(sys.argv[1], "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
proj = data.get("project", {})
print(json.dumps({
    "top_module": proj.get("top_module", ""),
    "tiles": proj.get("tiles", "8x2"),
    "source_files": proj.get("source_files", []),
}))
PYEOF
)"

TOP_MODULE="$("$TT_VENV_PY" -c "import json,sys; d=json.loads(sys.argv[1]); print(d['top_module'])" "$INFO_JSON")"
TILES="$("$TT_VENV_PY" -c "import json,sys; d=json.loads(sys.argv[1]); print(d['tiles'])" "$INFO_JSON")"

if [ -n "$TOP_OVERRIDE" ]; then
    TOP_MODULE="$TOP_OVERRIDE"
fi

SRC_FILES=()
if [ ${#FILE_OVERRIDES[@]} -gt 0 ]; then
    for f in "${FILE_OVERRIDES[@]}"; do
        if [[ "$f" = /* ]]; then
            SRC_FILES+=("$f")
        else
            SRC_FILES+=("$REPO_ROOT/$f")
        fi
    done
else
    while IFS= read -r line; do
        [ -n "$line" ] && SRC_FILES+=("$REPO_ROOT/src/$line")
    done < <("$TT_VENV_PY" -c "import json,sys; d=json.loads(sys.argv[1]); print('\n'.join(d['source_files']))" "$INFO_JSON")
fi

if [ -z "$TOP_MODULE" ] || [ ${#SRC_FILES[@]} -eq 0 ]; then
    echo "ERROR: Could not determine top_module or source_files." >&2
    exit 1
fi

RPT_PATH="/tmp/tt_inet_synth_sky130.rpt"

echo "=== Verilator Lint ($TOP_MODULE) ==="
LINT_STATUS="PASS"
if ! verilator --lint-only -Wall -I"$REPO_ROOT/src" --top-module "$TOP_MODULE" "${SRC_FILES[@]}" 2>&1; then
    LINT_STATUS="WARN/FAIL (see lint output above)"
fi

echo ""
echo "=== Yosys SKY130 HD Synthesis ($TOP_MODULE, target 50 MHz / 20ns) ==="
YOSYS_READ="read_verilog -sv -I$REPO_ROOT/src"
for f in "${SRC_FILES[@]}"; do
    YOSYS_READ="$YOSYS_READ \"$f\""
done

yosys -q -l "$RPT_PATH" -p "
$YOSYS_READ
hierarchy -check -top $TOP_MODULE
proc
flatten
opt_expr
opt_clean
check
opt -nodffe -nosdff
fsm
opt
wreduce
peepopt
opt_clean
alumacc
share
opt
memory -nomap
opt_clean
memory_map
opt -full
techmap
opt -fast
dfflibmap -liberty $SKY130_LIB
abc -liberty $SKY130_LIB -D 20000
opt_clean -purge
stat -liberty $SKY130_LIB
"

"$TT_VENV_PY" - "$RPT_PATH" "$TOP_MODULE" "$TILES" "$LINT_STATUS" <<'PYEOF'
import re, sys

rpt_path, top_mod, tiles, lint_status = sys.argv[1:5]
with open(rpt_path, "r", encoding="utf-8") as f:
    text = f.read()

# Find the final stat section
stat_idx = text.rfind("=== ")
stat_text = text[stat_idx:] if stat_idx != -1 else text

cells_match = re.search(r"^\s*(\d+)\s+[0-9.eE+-]+\s+cells\b", stat_text, re.MULTILINE)
if not cells_match:
    cells_match = re.search(r"Number of cells:\s+(\d+)", stat_text)
area_match = re.search(r"Chip area for module [^:]+:\s+([0-9.eE+-]+)", stat_text)
seq_area_match = re.search(r"of which used for sequential elements:\s+([0-9.eE+-]+)", stat_text)

total_cells = int(cells_match.group(1)) if cells_match else 0
chip_area = float(area_match.group(1)) if area_match else 0.0
seq_area = float(seq_area_match.group(1)) if seq_area_match else 0.0

seq_cells = 0
latch_cells = 0
for m in re.finditer(r"^\s*(\d+)\s+[0-9.eE+-]+\s+(sky130_fd_sc_hd__\S+)\s*$", stat_text, re.MULTILINE):
    cnt, cell = int(m.group(1)), m.group(2)
    if "__df" in cell:
        seq_cells += cnt
    elif "__dl" in cell:
        latch_cells += cnt
        seq_cells += cnt

comb_cells = total_cells - seq_cells
nand2_area = 3.7536  # sky130_fd_sc_hd__nand2_1 area in um^2
equiv_gates = chip_area / nand2_area

# 8x2 die: 1378.16 x 225.76 um -> 311,133 um^2, core ~ 302,420 um^2 (with margins)
core_area_8x2 = 302420.0
raw_util = (chip_area / core_area_8x2) * 100.0
post_cts_util = raw_util * 1.15

if raw_util <= 58.0:
    status = "PASS (fits 8x2 comfortably)"
elif raw_util <= 68.0:
    status = "WARN (tight for 8x2 CTS/routing)"
else:
    status = "FAIL (exceeds 8x2 safe utilization)"

print(f"Top module          : {top_mod} (info.yaml tiles: {tiles})")
print(f"Verilator lint      : {lint_status}")
print(f"Total cells         : {total_cells}")
print(f"  Sequential (FF/L) : {seq_cells} (latches: {latch_cells})")
print(f"  Combinational     : {comb_cells}")
print(f"Chip area (SKY130)  : {chip_area:,.2f} um^2 (seq: {seq_area:,.2f} um^2)")
print(f"Equiv NAND2 gates   : {equiv_gates:,.0f}")
print(f"8x2 core util (raw) : {raw_util:.2f}% of {core_area_8x2:,.0f} um^2")
print(f"8x2 est post-CTS    : {post_cts_util:.2f}% (x1.15 buffer/CTS headroom)")
print(f"Area check          : {status}")
print(f"Full Yosys report   : {rpt_path}")
PYEOF
