# ------------------------------------------------------------------------------
# robot_sumo.sdc : contraintes temporelles du robot SUMO (TimeQuest / Timing
# Analyzer de Quartus). Voir docs/COURS_FPGA-VHDL/COMPLEMENT_COURS/
# DOC_CONTRAINTES_TEMPORELLES/AnalyseTemporelleStatique.pdf.
# ------------------------------------------------------------------------------

# Oscillateur 50 MHz de la DE0-Nano (période 20 ns).
create_clock -name clk_50 -period 20.000 [get_ports {clock_50}]

# Horloge système 500 kHz : sortie de la bascule clk_500KHz_reg du diviseur de
# fréquence, 50 MHz divisé par 100. La déclarer permet à l'outil d'analyser
# les chemins cadencés par cette horloge et les passages 50 MHz -> 500 kHz.
create_generated_clock -name clk_500k -source [get_ports {clock_50}] -divide_by 100 \
  [get_registers {*u_diviseur_frequence|clk_500KHz_reg}]

derive_clock_uncertainty

# Entrées asynchrones (bouton, capteurs IR, ligne série) : resynchronisées par
# deux bascules dans le design, aucune contrainte de chemin à imposer.
set_false_path -from [get_ports {bp1_b ir_gauche_b ir_droit_b uart_rx}]

# Sorties lentes (LED, moteurs, ligne série) : pas de contrainte de temps.
set_false_path -to [get_ports {led[*] mot_gauche_dir mot_gauche_en mot_droit_dir mot_droit_en uart_tx}]

# Bus SPI de l'ADC, cadencé par le 50 MHz (SCLK généré par une bascule) :
# marges de carte prudentes.
set_input_delay  -clock clk_50 2.0 [get_ports {adc_sdat}]
set_output_delay -clock clk_50 2.0 [get_ports {adc_cs_n adc_saddr adc_sclk}]

# Les mesures ADC passent du domaine 50 MHz au domaine 500 kHz : horloges de
# même origine (relation de phase connue), l'analyse est faite normalement.
