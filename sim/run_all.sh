#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# run_all.sh : non-régression, rejoue tous les modules validés
# Source     : sim/modules.lst, une ligne par module validé, dans l'ordre :
#              <entite> [dependance ...]      (lignes vides et # ignorées)
# Code retour: 0 si tous passent, 1 sinon (liste des échecs affichée).
# -----------------------------------------------------------------------------
set -u
cd "$(dirname "$0")/.."
rm -rf sim/work sim/work93   # repart de bibliothèques propres
ECHECS=()
while read -r ligne; do
  case "$ligne" in ''|\#*) continue ;; esac
  # shellcheck disable=SC2086
  bash sim/run_tb.sh $ligne > /dev/null 2>&1 && echo "OK     $ligne" || { echo "ECHEC  $ligne"; ECHECS+=("$ligne"); }
done < sim/modules.lst
# Les UART fournis doivent garder la logique de l'enseignant (CDC §5).
bash sim/check_uart_origine.sh > /dev/null 2>&1 && echo "OK     UART identiques aux fichiers fournis" || { echo "ECHEC  UART modifiés par rapport aux fichiers fournis"; ECHECS+=("check_uart_origine"); }
[ ${#ECHECS[@]} -eq 0 ] && { echo "Non-régression : tout est vert."; exit 0; }
echo "Non-régression : ${#ECHECS[@]} échec(s)."; exit 1
