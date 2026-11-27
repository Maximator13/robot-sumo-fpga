#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# check_uart_origine.sh : garantit que UART_RX.vhd / UART_TX.vhd de src/ ont
# exactement la logique des fichiers fournis par l'enseignant (CDC §5) : on
# compare les deux versions privées de leurs commentaires et de leurs espaces.
# Code retour : 0 si identiques, 1 sinon.
# -----------------------------------------------------------------------------
cd "$(dirname "$0")/.."
norm() { sed -e 's/--.*$//' "$1" | tr -d ' \t\r\n'; }
RC=0
for f in UART_TX UART_RX; do
  if [ "$(norm "docs/Projet_FPGA_2026/VHDL_UART/$f.vhd")" = "$(norm "src/$f.vhd")" ]; then
    echo "OK     $f : logique identique au fichier fourni"
  else
    echo "ECHEC  $f : logique modifiée par rapport au fichier fourni"; RC=1
  fi
done
exit $RC
