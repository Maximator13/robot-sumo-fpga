# ------------------------------------------------------------------------------
# creer_projet.tcl : création du projet Quartus Prime Lite 20.1 du robot SUMO
# Usage (depuis le dossier quartus/) :
#   quartus_sh -t ../constraints/creer_projet.tcl
# Produit quartus/robot_sumo.qpf et quartus/robot_sumo.qsf : FPGA cible,
# fichiers VHDL (dans l'ordre des dépendances), contraintes de timing et
# affectation des broches de la DE0-Nano (manuel DE0-Nano et schéma de la carte
# d'interface robot carte_fpga_sumo_V4).
# ------------------------------------------------------------------------------
project_new robot_sumo -overwrite

# Cible : DE0-Nano, Cyclone IV E EP4CE22F17C6N.
set_global_assignment -name FAMILY "Cyclone IV E"
set_global_assignment -name DEVICE EP4CE22F17C6
set_global_assignment -name TOP_LEVEL_ENTITY robot_sumo_top
set_global_assignment -name VHDL_INPUT_VERSION VHDL_1993
set_global_assignment -name NUM_PARALLEL_PROCESSORS ALL
set_global_assignment -name STRATIX_DEVICE_IO_STANDARD "3.3-V LVTTL"
set_global_assignment -name RESERVE_ALL_UNUSED_PINS_WEAK_PULLUP "AS INPUT TRI-STATED"
set_global_assignment -name PROJECT_OUTPUT_DIRECTORY output_files

# Contraintes temporelles (analyse temporelle statique).
set_global_assignment -name SDC_FILE ../constraints/robot_sumo.sdc

# Sources VHDL : le paquetage d'abord, puis les modules, puis le top-level.
foreach fichier {
  sumo_pkg
  circuit_reset
  diviseur_frequence
  pwm_gen
  UART_RX
  UART_TX
  message_demarrage
  selection_mode
  adc128s022_ctrl
  detection_capteurs
  batterie_monitor
  fsm_combat
  suiveur_ligne
  robot_sumo_top
} {
  set_global_assignment -name VHDL_FILE ../src/$fichier.vhd
}

# Broches (schéma carte_fpga_sumo_V4 + manuel DE0-Nano, tables 3-2, 3-8, 3-9).
set_location_assignment PIN_R8  -to clock_50
set_location_assignment PIN_G15 -to bp1_b
set_location_assignment PIN_A14 -to ir_gauche_b
set_location_assignment PIN_C14 -to ir_droit_b
set_location_assignment PIN_D16 -to uart_rx
set_location_assignment PIN_D14 -to uart_tx
set_location_assignment PIN_C15 -to mot_gauche_dir
set_location_assignment PIN_F15 -to mot_gauche_en
set_location_assignment PIN_D15 -to mot_droit_dir
set_location_assignment PIN_F14 -to mot_droit_en
set_location_assignment PIN_A10 -to adc_cs_n
set_location_assignment PIN_B10 -to adc_saddr
set_location_assignment PIN_A9  -to adc_sdat
set_location_assignment PIN_B14 -to adc_sclk
set_location_assignment PIN_A15 -to led[0]
set_location_assignment PIN_A13 -to led[1]
set_location_assignment PIN_B13 -to led[2]
set_location_assignment PIN_A11 -to led[3]
set_location_assignment PIN_D1  -to led[4]
set_location_assignment PIN_F3  -to led[5]
set_location_assignment PIN_B1  -to led[6]
set_location_assignment PIN_L3  -to led[7]

# La sortie série de UART_TX (code fourni, sans reset ni valeur initiale) est
# masquée à '1' pendant le reset par robot_sumo_top : aucune affectation de
# niveau de mise sous tension n'est nécessaire.

project_close
