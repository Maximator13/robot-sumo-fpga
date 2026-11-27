#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# run_tb.sh : valide un module VHDL (simulation, compatibilité, latch)
# Usage     : sim/run_tb.sh <entite> [dependance ...]
#   <entite>      : nom de l'entité de src/ (le testbench est tb/tb_<entite>.vhd)
#   [dependance]  : autres entités nécessaires, cherchées dans src/ puis tb/
#                   (ex. modèle de simulation tb/adc128s022_model.vhd)
# Enchaîne      : analyse VHDL-2008 -> élaboration -> analyse VHDL-93 de src/
#                 (compatibilité Quartus) -> simulation (assertion ERROR = échec)
#                 -> présence de "TEST OK" -> ghdl --synth (refuse tout latch)
# Code retour   : 0 si le module est validé, 1 sinon.
# -----------------------------------------------------------------------------
set -u
cd "$(dirname "$0")/.."

# Localise GHDL : variable GHDL, sinon PATH, sinon installation winget.
GHDL="${GHDL:-$(command -v ghdl || true)}"
if [ -z "$GHDL" ]; then
  GHDL="$(ls /c/Users/*/AppData/Local/Microsoft/WinGet/Packages/ghdl.ghdl.*/bin/ghdl.exe 2>/dev/null | head -1)"
fi
[ -x "$GHDL" ] || { echo "ECHEC : GHDL introuvable (installer GHDL ou définir GHDL=...)"; exit 1; }

[ $# -ge 1 ] || { echo "Usage : sim/run_tb.sh <entite> [dependance ...]"; exit 1; }
ENT="$1"; shift
# Une bibliothèque par module : les validations peuvent tourner en parallèle.
WORK="sim/work/${ENT}"; LOG="sim/logs/tb_${ENT}.log"
mkdir -p "$WORK" sim/logs
OPTS=(--std=08 --workdir="$WORK")

# Fichier source d'une entité : src/ en priorité, puis tb/.
src_of() { for d in src tb; do [ -f "$d/$1.vhd" ] && { echo "$d/$1.vhd"; return; }; done; echo "ECHEC : $1.vhd introuvable" >&2; exit 1; }

FILES=()
[ -f src/sumo_pkg.vhd ] && FILES+=(src/sumo_pkg.vhd)
for dep in "$@"; do FILES+=("$(src_of "$dep")"); done
[ "$ENT" != "sumo_pkg" ] && FILES+=("$(src_of "$ENT")")
FILES+=("tb/tb_${ENT}.vhd")

echo "== Analyse : ${FILES[*]}"
"$GHDL" -a "${OPTS[@]}" "${FILES[@]}" || { echo "ECHEC analyse ($ENT)"; exit 1; }
"$GHDL" -e "${OPTS[@]}" "tb_${ENT}" || { echo "ECHEC élaboration ($ENT)"; exit 1; }

# Compatibilité Quartus (projet en VHDL_1993) : les fichiers de src/ seuls
# doivent aussi s'analyser en VHDL-93. Les testbenches restent en VHDL-2008.
SRC93=()
for f in "${FILES[@]}"; do case "$f" in src/*) SRC93+=("$f") ;; esac; done
if [ ${#SRC93[@]} -gt 0 ]; then
  echo "== Analyse VHDL-93 (compatibilité Quartus) : ${SRC93[*]}"
  mkdir -p "sim/work93/${ENT}"
  "$GHDL" -a --std=93c --workdir="sim/work93/${ENT}" "${SRC93[@]}" || { echo "ECHEC : src/ non compatible VHDL-93 ($ENT)"; exit 1; }
fi

echo "== Simulation : tb_${ENT}"
# Garde-fou en temps réel (en plus du chien de garde du testbench) :
# SIM_TIMEOUT secondes au maximum, 300 par défaut.
timeout "${SIM_TIMEOUT:-300}" "$GHDL" -r "${OPTS[@]}" "tb_${ENT}" --assert-level=error --wave="sim/tb_${ENT}.ghw" > "$LOG" 2>&1
RC=$?
cat "$LOG"
[ $RC -eq 124 ] && { echo "ECHEC : simulation bloquée, délai de ${SIM_TIMEOUT:-300} s dépassé ($ENT)"; exit 1; }
[ $RC -eq 0 ] || { echo "ECHEC simulation ($ENT), code $RC"; exit 1; }
grep -q "TEST OK" "$LOG" || { echo "ECHEC : 'TEST OK' absent du log ($ENT)"; exit 1; }

# Contrôle synthétisable + zéro latch sur CHAQUE entité de src/ analysée
# (module testé et ses dépendances ; le paquetage n'est pas une entité).
for f in "${SRC93[@]}"; do
  E="$(basename "$f" .vhd)"
  [ "$E" = "sumo_pkg" ] && continue
  echo "== Synthèse GHDL (contrôle latch) : $E"
  "$GHDL" --synth "${OPTS[@]}" "$E" > /dev/null 2> "sim/logs/synth_${E}.log"
  RC=$?
  cat "sim/logs/synth_${E}.log"
  [ $RC -eq 0 ] || { echo "ECHEC synthèse/latch ($E)"; exit 1; }
  grep -qi "latch" "sim/logs/synth_${E}.log" && { echo "ECHEC : latch détecté ($E)"; exit 1; }
done

echo "VALIDE : $ENT"
