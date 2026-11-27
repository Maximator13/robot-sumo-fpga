# ------------------------------------------------------------------------------
# simuler.do : simulation de tous les bancs de test du robot SUMO avec
#              ModelSim (livré avec Quartus Prime Lite 20.1).
# Utilisation : dans ModelSim, File > Change Directory vers le dossier sim/ du
#              projet, puis taper dans la console : do simuler.do
# Principe   : compile l'IP de l'ADC (Verilog généré par Quartus), les sources
#              VHDL de src/ et les bancs de tb/, puis lance chaque banc. Chaque
#              banc vérifie lui-même ses résultats et affiche "TEST OK" ou
#              "TEST KO" dans la console. Le bilan est affiché à la fin.
# ------------------------------------------------------------------------------

# Dossier du projet : parent du dossier sim/ dans lequel on se trouve.
if {[file tail [pwd]] ne "sim"} {
  error "Se placer d'abord dans le dossier sim/ du projet (File > Change Directory)."
}
set projet [file normalize ..]
set ip  $projet/ip/adc_ctrl/synthesis
set src $projet/src
set tb  $projet/tb

# Bibliothèque de travail dans sim/, recréée à chaque lancement.
cd $projet/sim
if {[file exists work]} { vdel -all -lib work }
vlib work

# IP de l'ADC : deux fichiers Verilog puis son enveloppe VHDL.
vlog -quiet $ip/submodules/altera_up_avalon_adv_adc.v $ip/submodules/adc_ctrl_adc_mega_0.v
vcom -2008 -quiet $ip/adc_ctrl.vhd

# Sources VHDL, dans l'ordre des dépendances.
foreach f {circuit_reset diviseur_frequence pwm_gen UART_RX UART_TX
           message_demarrage selection_mode detection_capteurs batterie_monitor
           fsm_combat suiveur_ligne robot_sumo_top} {
  vcom -2008 -quiet $src/$f.vhd
}

# Modèle de l'ADC128S022 (simulation seulement) et bancs de test.
vcom -2008 -quiet $tb/adc128s022_model.vhd
set bancs {tb_diviseur_frequence tb_circuit_reset tb_pwm_gen
           tb_UART_RX tb_message_demarrage tb_selection_mode
           tb_detection_capteurs tb_batterie_monitor tb_fsm_combat
           tb_suiveur_ligne tb_robot_sumo_top}
foreach b $bancs { vcom -2008 -quiet $tb/$b.vhd }

# Lancement de chaque banc ; le verdict est lu dans la console (transcript).
set bilan {}
foreach b $bancs {
  transcript file $projet/sim/$b.log
  vsim -quiet -onfinish stop -voptargs=+acc work.$b
  onbreak {resume}
  run -all
  quit -sim
  transcript file ""
  set fichier [open $projet/sim/$b.log r]; set texte [read $fichier]; close $fichier
  if {[string match "*TEST OK*" $texte] && ![string match "*TEST KO*" $texte]} {
    lappend bilan "OK     $b"
  } else {
    lappend bilan "ECHEC  $b (voir sim/$b.log)"
  }
}
echo "---------------- Bilan des bancs de test ----------------"
foreach ligne $bilan { echo $ligne }
