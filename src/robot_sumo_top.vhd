--------------------------------------------------------------------------------
-- Fichier      : robot_sumo_top.vhd
-- Entité       : robot_sumo_top
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Niveau le plus haut du robot SUMO : relie entre eux tous les
--                sous-systèmes et les broches de la carte DE0-Nano / carte
--                d'interface robot. Le robot fonctionne en deux modes choisis
--                depuis le PC par la liaison série : 'c' = combat SUMO,
--                's' = suiveur de ligne (course EMA). À la mise sous tension
--                il envoie "Robot PRET !" au PC (cahier des charges §1).
-- Principe     : Description structurelle, comme au §7 du cours VHDL :
--                chaque sous-système est déclaré par un COMPONENT, puis
--                instancié (label : composant PORT MAP) et relié aux autres
--                par des signaux s_.... Quand le sous-système a des
--                paramètres (durées en cycles), ils lui sont passés par
--                GENERIC MAP, ce qui permet au banc de test de les réduire.
--                L'ADC est lu par l'IP « ADC Controller for DE-series
--                Boards » du catalogue IP de Quartus (même IP que le TP2,
--                composant adc_ctrl, fichiers dans ip/adc_ctrl).
--                Deux horloges :
--                - 50 MHz (oscillateur) : circuit de reset, diviseur de
--                  fréquence, IP de l'ADC ;
--                - 500 kHz (sortie du diviseur, CDC §6.2) : tout le reste.
--                Seule logique propre au top-level :
--                - l'aiguillage des commandes moteurs : celles de la machine
--                  de combat en mode combat, sinon celles du suiveur de ligne
--                  (qui commande l'arrêt quand il n'est pas actif) ;
--                - la recopie des sens moteurs et des LED dans des bascules
--                  (sorties sans parasite) ;
--                - la ligne série forcée au repos ('1') pendant le reset et
--                  les 2 cycles suivants, car l'émetteur UART_TX fourni n'a
--                  ni reset ni valeur de mise sous tension et son horloge est
--                  arrêtée pendant le reset.
--                - l'adaptation de l'IP de l'ADC : son reset est actif HAUT
--                  (on lui envoie NOT rst_b) et ses 8 sorties CH0 à CH7
--                  (STD_LOGIC_VECTOR) sont rangées dans le tableau s_voies ;
--                Les sorties de mise au point des sous-systèmes (envoye de
--                message_demarrage, etat_dbg des deux machines à états) ne
--                servent qu'en simulation : elles sont laissées ouvertes
--                (OPEN) ici.
--                LED : led(7) = niveau batterie (CDC §6.4) ; led(6 DOWNTO 5)
--                = mode ("00" attente, "01" combat, "10" suiveur) ;
--                led(4 DOWNTO 0) = capteurs de sol vus noirs (aide au réglage
--                du seuil noir / blanc).
-- Entrées      : clock_50   : oscillateur 50 MHz (PIN_R8).
--                bp1_b      : bouton BP1 de la carte d'interface (reset).
--                ir_gauche_b, ir_droit_b : capteurs d'obstacle, '0' = obstacle.
--                uart_rx    : ligne série venant du PC.
--                adc_sdat   : donnée série de l'ADC.
-- Sorties      : uart_tx    : ligne série vers le PC.
--                mot_gauche_dir, mot_droit_dir : sens des moteurs.
--                mot_gauche_en, mot_droit_en   : PWM 5 kHz des moteurs.
--                adc_cs_n, adc_saddr, adc_sclk : bus SPI de l'ADC.
--                led        : 8 LED de la DE0-Nano.
-- Génériques   : durées en cycles, valeurs réelles par défaut (réduites par
--                le banc de test) : RESET_CYCLES_G (1 ms à 50 MHz),
--                ATTENTE_COMBAT_CYCLES_G (5 s), CLIGNO_DEMI_CYCLES_G (0,5 s),
--                FILTRE_BATT_CYCLES_G (1 s), BORDURE_MAINTIEN_CYCLES_G
--                (300 ms), FIN_LIGNE_CYCLES_G (200 ms), à 500 kHz sauf mention.
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY robot_sumo_top IS
  GENERIC (
    RESET_CYCLES_G            : NATURAL := RESET_CYCLES;
    ATTENTE_COMBAT_CYCLES_G   : NATURAL := ATTENTE_COMBAT_CYCLES;
    CLIGNO_DEMI_CYCLES_G      : NATURAL := CLIGNO_DEMI_CYCLES;
    FILTRE_BATT_CYCLES_G      : NATURAL := FILTRE_BATT_CYCLES;
    BORDURE_MAINTIEN_CYCLES_G : NATURAL := BORDURE_MAINTIEN_CYCLES;
    FIN_LIGNE_CYCLES_G        : NATURAL := FIN_LIGNE_CYCLES
  );
  PORT (
    clock_50       : IN  STD_LOGIC;                      -- oscillateur 50 MHz
    bp1_b          : IN  STD_LOGIC;                      -- bouton BP1, actif à '0'
    ir_gauche_b    : IN  STD_LOGIC;                      -- obstacle à gauche si '0'
    ir_droit_b     : IN  STD_LOGIC;                      -- obstacle à droite si '0'
    uart_rx        : IN  STD_LOGIC;                      -- ligne série PC -> robot
    uart_tx        : OUT STD_LOGIC;                      -- ligne série robot -> PC
    mot_gauche_dir : OUT STD_LOGIC;                      -- sens du moteur gauche
    mot_gauche_en  : OUT STD_LOGIC;                      -- PWM du moteur gauche
    mot_droit_dir  : OUT STD_LOGIC;                      -- sens du moteur droit
    mot_droit_en   : OUT STD_LOGIC;                      -- PWM du moteur droit
    adc_cs_n       : OUT STD_LOGIC;                      -- sélection de l'ADC
    adc_saddr      : OUT STD_LOGIC;                      -- donnée vers l'ADC
    adc_sclk       : OUT STD_LOGIC;                      -- horloge SPI de l'ADC
    adc_sdat       : IN  STD_LOGIC;                      -- donnée venant de l'ADC
    led            : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)    -- LED de la DE0-Nano
  );
END ENTITY robot_sumo_top;

ARCHITECTURE robot_sumo_top_arch OF robot_sumo_top IS

  -- Reset général et horloge système.
  SIGNAL s_rst_b         : STD_LOGIC;
  SIGNAL s_clk_500k      : STD_LOGIC;

  -- IP de l'ADC : reset actif haut et sorties brutes des 8 voies.
  SIGNAL s_rst_adc       : STD_LOGIC;
  SIGNAL s_ch0, s_ch1, s_ch2, s_ch3 : STD_LOGIC_VECTOR(11 DOWNTO 0);
  SIGNAL s_ch4, s_ch5, s_ch6, s_ch7 : STD_LOGIC_VECTOR(11 DOWNTO 0);

  -- Mesures de l'ADC et grandeurs déduites.
  SIGNAL s_voies         : t_voies_adc;
  SIGNAL s_sol_noir      : STD_LOGIC_VECTOR(4 DOWNTO 0);
  SIGNAL s_bordure       : STD_LOGIC;
  SIGNAL s_adv_gauche    : STD_LOGIC;
  SIGNAL s_adv_droite    : STD_LOGIC;
  SIGNAL s_adversaire    : STD_LOGIC;
  SIGNAL s_code_batterie : t_mot_adc;
  SIGNAL s_led7          : STD_LOGIC;
  SIGNAL s_batt_vide     : STD_LOGIC;

  -- Liaison série.
  SIGNAL s_rx_dv         : STD_LOGIC;
  SIGNAL s_rx_octet      : STD_LOGIC_VECTOR(7 DOWNTO 0);
  SIGNAL s_tx_dv         : STD_LOGIC;
  SIGNAL s_tx_octet      : STD_LOGIC_VECTOR(7 DOWNTO 0);
  SIGNAL s_tx_actif      : STD_LOGIC;
  SIGNAL s_tx_serie      : STD_LOGIC;
  SIGNAL s_tx_fini       : STD_LOGIC;
  -- Masque de la ligne TX : "11" deux cycles après la fin du reset.
  SIGNAL s_tx_libre      : STD_LOGIC_VECTOR(1 DOWNTO 0);

  -- Mode de fonctionnement.
  SIGNAL s_mode          : t_mode;
  SIGNAL s_combat_actif  : STD_LOGIC;
  SIGNAL s_suiveur_actif : STD_LOGIC;

  -- Commandes moteurs de chaque comportement.
  SIGNAL s_combat_dir_g, s_combat_dir_d   : STD_LOGIC;
  SIGNAL s_combat_duty_g, s_combat_duty_d : t_duty;
  SIGNAL s_suiv_dir_g, s_suiv_dir_d       : STD_LOGIC;
  SIGNAL s_suiv_duty_g, s_suiv_duty_d     : t_duty;

  -- Commandes moteurs retenues (après aiguillage).
  SIGNAL s_dir_g, s_dir_d   : STD_LOGIC;
  SIGNAL s_duty_g, s_duty_d : t_duty;

  -- Bascules des sorties sens moteurs et LED.
  SIGNAL s_mot_gauche_dir_reg : STD_LOGIC;
  SIGNAL s_mot_droit_dir_reg  : STD_LOGIC;
  SIGNAL s_led_reg            : STD_LOGIC_VECTOR(7 DOWNTO 0);
  SIGNAL s_code_mode          : STD_LOGIC_VECTOR(1 DOWNTO 0);

  ------------------------------------------------------------------------------
  -- Déclaration des composants (cours VHDL §7.1). Chaque composant porte le
  -- nom de son entité : Quartus les associe sans configuration (cours §8).
  ------------------------------------------------------------------------------
  -- Reset général : rst_b à '0' pendant 1 ms (mise sous tension, BP1).
  COMPONENT circuit_reset IS
    GENERIC (RESET_CYCLES_G : NATURAL);
    PORT (clk_in : IN  STD_LOGIC;
          bp_in  : IN  STD_LOGIC;
          rst_b  : OUT STD_LOGIC);
  END COMPONENT circuit_reset;

  -- Horloge système 500 kHz obtenue en divisant le 50 MHz par 100.
  COMPONENT diviseur_frequence IS
    PORT (clk_in     : IN  STD_LOGIC;
          rst_in_b   : IN  STD_LOGIC;
          clk_500KHz : OUT STD_LOGIC);
  END COMPONENT diviseur_frequence;

  -- IP « ADC Controller for DE-series Boards » (catalogue IP de Quartus,
  -- University Program, comme au TP2) : lit en boucle les 8 voies de
  -- l'ADC128S022 par le bus SPI. Déclaration recopiée de ip/adc_ctrl/adc_ctrl.cmp.
  COMPONENT adc_ctrl IS
    PORT (CLOCK    : IN  STD_LOGIC;
          ADC_SCLK : OUT STD_LOGIC;
          ADC_CS_N : OUT STD_LOGIC;
          ADC_DOUT : IN  STD_LOGIC;
          ADC_DIN  : OUT STD_LOGIC;
          CH0      : OUT STD_LOGIC_VECTOR(11 DOWNTO 0);
          CH1      : OUT STD_LOGIC_VECTOR(11 DOWNTO 0);
          CH2      : OUT STD_LOGIC_VECTOR(11 DOWNTO 0);
          CH3      : OUT STD_LOGIC_VECTOR(11 DOWNTO 0);
          CH4      : OUT STD_LOGIC_VECTOR(11 DOWNTO 0);
          CH5      : OUT STD_LOGIC_VECTOR(11 DOWNTO 0);
          CH6      : OUT STD_LOGIC_VECTOR(11 DOWNTO 0);
          CH7      : OUT STD_LOGIC_VECTOR(11 DOWNTO 0);
          RESET    : IN  STD_LOGIC);
  END COMPONENT adc_ctrl;

  -- Interprétation des capteurs : sol noir / clair, bordure, obstacles.
  COMPONENT detection_capteurs IS
    GENERIC (BORDURE_MAINTIEN_CYCLES_G : NATURAL);
    PORT (clk           : IN  STD_LOGIC;
          rst_b         : IN  STD_LOGIC;
          voies         : IN  t_voies_adc;
          ir_gauche_b   : IN  STD_LOGIC;
          ir_droit_b    : IN  STD_LOGIC;
          sol_noir      : OUT STD_LOGIC_VECTOR(4 DOWNTO 0);
          bordure       : OUT STD_LOGIC;
          adv_gauche    : OUT STD_LOGIC;
          adv_droite    : OUT STD_LOGIC;
          adversaire    : OUT STD_LOGIC;
          code_batterie : OUT t_mot_adc);
  END COMPONENT detection_capteurs;

  -- Surveillance batterie : LED7 en PWM ou clignotante, signal batt_vide.
  COMPONENT batterie_monitor IS
    GENERIC (CLIGNO_DEMI_CYCLES_G : NATURAL;
             FILTRE_BATT_CYCLES_G : NATURAL);
    PORT (clk           : IN  STD_LOGIC;
          rst_b         : IN  STD_LOGIC;
          code_batterie : IN  t_mot_adc;
          led7          : OUT STD_LOGIC;
          batt_vide     : OUT STD_LOGIC);
  END COMPONENT batterie_monitor;

  -- Récepteur et émetteur série fournis (entités UART_RX et UART_TX,
  -- nandland). VHDL ne distingue pas majuscules et minuscules : un composant
  -- nommé UART_RX entrerait en conflit avec le port uart_rx du robot. Les
  -- composants portent donc un autre nom (recepteur_serie, emetteur_serie) et
  -- sont reliés aux entités fournies par une configuration (cours VHDL §8 :
  -- « le nom d'un composant peut être différent de celui de l'entité
  -- associée »), voir les lignes FOR ... USE ENTITY ci-dessous.
  COMPONENT recepteur_serie IS
    GENERIC (g_CLKS_PER_BIT : INTEGER);
    PORT (i_Clk       : IN  STD_LOGIC;
          i_RX_Serial : IN  STD_LOGIC;
          o_RX_DV     : OUT STD_LOGIC;
          o_RX_Byte   : OUT STD_LOGIC_VECTOR(7 DOWNTO 0));
  END COMPONENT recepteur_serie;

  COMPONENT emetteur_serie IS
    GENERIC (g_CLKS_PER_BIT : INTEGER);
    PORT (i_Clk       : IN  STD_LOGIC;
          i_TX_DV     : IN  STD_LOGIC;
          i_TX_Byte   : IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
          o_TX_Active : OUT STD_LOGIC;
          o_TX_Serial : OUT STD_LOGIC;
          o_TX_Done   : OUT STD_LOGIC);
  END COMPONENT emetteur_serie;

  -- Configuration : chaque instance de ces composants utilise l'entité
  -- fournie par l'enseignant.
  FOR u_UART_RX : recepteur_serie USE ENTITY WORK.UART_RX;
  FOR u_UART_TX : emetteur_serie  USE ENTITY WORK.UART_TX;

  -- Envoi du message "Robot PRET !\n\r" après chaque reset.
  COMPONENT message_demarrage IS
    PORT (clk      : IN  STD_LOGIC;
          rst_b    : IN  STD_LOGIC;
          tx_actif : IN  STD_LOGIC;
          tx_fini  : IN  STD_LOGIC;
          tx_dv    : OUT STD_LOGIC;
          tx_octet : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
          envoye   : OUT STD_LOGIC);
  END COMPONENT message_demarrage;

  -- Choix du mode par les caractères 'c' et 's' reçus du PC.
  COMPONENT selection_mode IS
    PORT (clk      : IN  STD_LOGIC;
          rst_b    : IN  STD_LOGIC;
          rx_dv    : IN  STD_LOGIC;
          rx_octet : IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
          mode     : OUT t_mode);
  END COMPONENT selection_mode;

  -- Machine à états du combat SUMO (CDC §6.3).
  COMPONENT fsm_combat IS
    GENERIC (ATTENTE_CYCLES_G : NATURAL);
    PORT (clk         : IN  STD_LOGIC;
          rst_b       : IN  STD_LOGIC;
          actif       : IN  STD_LOGIC;
          adversaire  : IN  STD_LOGIC;
          adv_gauche  : IN  STD_LOGIC;
          adv_droite  : IN  STD_LOGIC;
          bordure     : IN  STD_LOGIC;
          batt_vide   : IN  STD_LOGIC;
          dir_gauche  : OUT STD_LOGIC;
          dir_droit   : OUT STD_LOGIC;
          duty_gauche : OUT t_duty;
          duty_droit  : OUT t_duty;
          etat_dbg    : OUT STD_LOGIC_VECTOR(2 DOWNTO 0));
  END COMPONENT fsm_combat;

  -- Machine à états du suiveur de ligne (course EMA, CDC §4).
  COMPONENT suiveur_ligne IS
    GENERIC (FIN_LIGNE_CYCLES_G : NATURAL);
    PORT (clk         : IN  STD_LOGIC;
          rst_b       : IN  STD_LOGIC;
          actif       : IN  STD_LOGIC;
          sol_noir    : IN  STD_LOGIC_VECTOR(4 DOWNTO 0);
          batt_vide   : IN  STD_LOGIC;
          dir_gauche  : OUT STD_LOGIC;
          dir_droit   : OUT STD_LOGIC;
          duty_gauche : OUT t_duty;
          duty_droit  : OUT t_duty;
          etat_dbg    : OUT STD_LOGIC_VECTOR(2 DOWNTO 0));
  END COMPONENT suiveur_ligne;

  -- Générateur PWM 5 kHz (un par moteur).
  COMPONENT pwm_gen IS
    PORT (clk   : IN  STD_LOGIC;
          rst_b : IN  STD_LOGIC;
          duty  : IN  t_duty;
          pwm   : OUT STD_LOGIC);
  END COMPONENT pwm_gen;

BEGIN

  ------------------------------------------------------------------------------
  -- Horloges et reset (50 MHz)
  ------------------------------------------------------------------------------
  -- rst_b à '0' pendant 1 ms à la mise sous tension et après un appui sur BP1.
  u_circuit_reset : circuit_reset
    GENERIC MAP (RESET_CYCLES_G => RESET_CYCLES_G)
    PORT MAP (clk_in => clock_50, bp_in => bp1_b, rst_b => s_rst_b);

  -- Horloge système 500 kHz, figée à '0' pendant le reset.
  u_diviseur_frequence : diviseur_frequence
    PORT MAP (clk_in => clock_50, rst_in_b => s_rst_b, clk_500KHz => s_clk_500k);

  ------------------------------------------------------------------------------
  -- Acquisition : ADC (50 MHz) puis interprétation des capteurs (500 kHz)
  ------------------------------------------------------------------------------
  -- Lecture en boucle des 8 voies de l'ADC128S022 (batterie + capteurs de sol)
  -- par l'IP du TP2, cadencée à 50 MHz (SCLK = 50 MHz / 16 = 3,125 MHz, sous
  -- le maximum de 3,2 MHz de la datasheet).
  -- Le reset de l'IP est actif HAUT : on lui envoie l'inverse de rst_b.
  s_rst_adc <= NOT s_rst_b;

  u_adc_ctrl : adc_ctrl
    PORT MAP (CLOCK    => clock_50,
              RESET    => s_rst_adc,
              ADC_SCLK => adc_sclk,
              ADC_CS_N => adc_cs_n,
              ADC_DOUT => adc_sdat,     -- résultat série envoyé par l'ADC
              ADC_DIN  => adc_saddr,    -- adresse de la voie envoyée à l'ADC
              CH0 => s_ch0, CH1 => s_ch1, CH2 => s_ch2, CH3 => s_ch3,
              CH4 => s_ch4, CH5 => s_ch5, CH6 => s_ch6, CH7 => s_ch7);

  -- Rangement des 8 mesures dans le tableau des voies (indice = numéro de
  -- l'entrée analogique Analog_In0 à Analog_In7), converties en nombres non
  -- signés pour les comparaisons de seuil.
  s_voies(0) <= UNSIGNED(s_ch0);
  s_voies(1) <= UNSIGNED(s_ch1);
  s_voies(2) <= UNSIGNED(s_ch2);
  s_voies(3) <= UNSIGNED(s_ch3);
  s_voies(4) <= UNSIGNED(s_ch4);
  s_voies(5) <= UNSIGNED(s_ch5);
  s_voies(6) <= UNSIGNED(s_ch6);
  s_voies(7) <= UNSIGNED(s_ch7);

  -- Sol noir / clair, bordure du Dohyo, obstacles, code batterie.
  u_detection_capteurs : detection_capteurs
    GENERIC MAP (BORDURE_MAINTIEN_CYCLES_G => BORDURE_MAINTIEN_CYCLES_G)
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b, voies => s_voies,
              ir_gauche_b => ir_gauche_b, ir_droit_b => ir_droit_b,
              sol_noir => s_sol_noir, bordure => s_bordure,
              adv_gauche => s_adv_gauche, adv_droite => s_adv_droite,
              adversaire => s_adversaire, code_batterie => s_code_batterie);

  -- Niveau de batterie sur LED7, détection de batterie vide (CDC §6.4).
  u_batterie_monitor : batterie_monitor
    GENERIC MAP (CLIGNO_DEMI_CYCLES_G => CLIGNO_DEMI_CYCLES_G,
                 FILTRE_BATT_CYCLES_G => FILTRE_BATT_CYCLES_G)
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b, code_batterie => s_code_batterie,
              led7 => s_led7, batt_vide => s_batt_vide);

  ------------------------------------------------------------------------------
  -- Liaison série avec le PC (9600 bauds 8N1, code UART fourni)
  ------------------------------------------------------------------------------
  u_UART_RX : recepteur_serie
    GENERIC MAP (g_CLKS_PER_BIT => UART_CLKS_PER_BIT)
    PORT MAP (i_Clk => s_clk_500k, i_RX_Serial => uart_rx,
              o_RX_DV => s_rx_dv, o_RX_Byte => s_rx_octet);

  u_UART_TX : emetteur_serie
    GENERIC MAP (g_CLKS_PER_BIT => UART_CLKS_PER_BIT)
    PORT MAP (i_Clk => s_clk_500k, i_TX_DV => s_tx_dv, i_TX_Byte => s_tx_octet,
              o_TX_Active => s_tx_actif, o_TX_Serial => s_tx_serie, o_TX_Done => s_tx_fini);

  -- Message "Robot PRET !\n\r" envoyé une fois après chaque reset.
  u_message_demarrage : message_demarrage
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b, tx_actif => s_tx_actif, tx_fini => s_tx_fini,
              tx_dv => s_tx_dv, tx_octet => s_tx_octet, envoye => OPEN);

  -- Ligne série au repos ('1') pendant le reset et les 2 premiers cycles qui
  -- suivent : UART_TX (code fourni) n'a ni reset ni valeur de mise sous
  -- tension sur o_TX_Serial, et son horloge est arrêtée pendant le reset ; il
  -- ne met sa sortie au repos qu'au premier front d'horloge. Sans ce masque,
  -- le PC recevrait un faux octet à la mise sous tension.
  validation_tx : PROCESS (s_clk_500k, s_rst_b)
  BEGIN
    IF s_rst_b = '0' THEN
      s_tx_libre <= "00";
    ELSIF RISING_EDGE(s_clk_500k) THEN
      s_tx_libre <= s_tx_libre(0) & '1';
    END IF;
  END PROCESS validation_tx;

  uart_tx <= s_tx_serie OR NOT s_tx_libre(1);

  -- Choix du mode par le PC : 'c' combat, 's' suiveur.
  u_selection_mode : selection_mode
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b, rx_dv => s_rx_dv,
              rx_octet => s_rx_octet, mode => s_mode);

  s_combat_actif  <= '1' WHEN s_mode = MODE_COMBAT  ELSE '0';
  s_suiveur_actif <= '1' WHEN s_mode = MODE_SUIVEUR ELSE '0';

  ------------------------------------------------------------------------------
  -- Comportements (machines à états)
  ------------------------------------------------------------------------------
  -- Combat SUMO (diagramme imposé, CDC §6.3).
  u_fsm_combat : fsm_combat
    GENERIC MAP (ATTENTE_CYCLES_G => ATTENTE_COMBAT_CYCLES_G)
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b, actif => s_combat_actif,
              adversaire => s_adversaire, adv_gauche => s_adv_gauche, adv_droite => s_adv_droite,
              bordure => s_bordure, batt_vide => s_batt_vide,
              dir_gauche => s_combat_dir_g, dir_droit => s_combat_dir_d,
              duty_gauche => s_combat_duty_g, duty_droit => s_combat_duty_d,
              etat_dbg => OPEN);

  -- Suiveur de ligne (course EMA, CDC §4).
  u_suiveur_ligne : suiveur_ligne
    GENERIC MAP (FIN_LIGNE_CYCLES_G => FIN_LIGNE_CYCLES_G)
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b, actif => s_suiveur_actif,
              sol_noir => s_sol_noir, batt_vide => s_batt_vide,
              dir_gauche => s_suiv_dir_g, dir_droit => s_suiv_dir_d,
              duty_gauche => s_suiv_duty_g, duty_droit => s_suiv_duty_d,
              etat_dbg => OPEN);

  ------------------------------------------------------------------------------
  -- Aiguillage des commandes moteurs et PWM 5 kHz
  ------------------------------------------------------------------------------
  -- En mode combat : machine de combat ; sinon : suiveur de ligne, qui
  -- commande l'arrêt (duty = 0) quand il n'est pas actif (mode attente).
  s_dir_g  <= s_combat_dir_g  WHEN s_mode = MODE_COMBAT ELSE s_suiv_dir_g;
  s_dir_d  <= s_combat_dir_d  WHEN s_mode = MODE_COMBAT ELSE s_suiv_dir_d;
  s_duty_g <= s_combat_duty_g WHEN s_mode = MODE_COMBAT ELSE s_suiv_duty_g;
  s_duty_d <= s_combat_duty_d WHEN s_mode = MODE_COMBAT ELSE s_suiv_duty_d;

  -- PWM 5 kHz sur les entrées ENABLE du pont en H : '0' = moteur arrêté.
  u_pwm_gauche : pwm_gen
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b, duty => s_duty_g, pwm => mot_gauche_en);

  u_pwm_droit : pwm_gen
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b, duty => s_duty_d, pwm => mot_droit_en);

  ------------------------------------------------------------------------------
  -- Sorties sens moteurs et LED, recopiées dans des bascules
  ------------------------------------------------------------------------------
  -- Code du mode affiché sur led(6 DOWNTO 5).
  s_code_mode <= "01" WHEN s_mode = MODE_COMBAT ELSE
                 "10" WHEN s_mode = MODE_SUIVEUR ELSE
                 "00";

  registre_sorties : PROCESS (s_clk_500k, s_rst_b)
  BEGIN
    IF s_rst_b = '0' THEN
      -- Reset : sens « avancer » (moteurs de toute façon arrêtés par les PWM
      -- à '0'), LED éteintes.
      s_mot_gauche_dir_reg <= DIR_AVANT_GAUCHE;
      s_mot_droit_dir_reg  <= DIR_AVANT_DROIT;
      s_led_reg            <= (OTHERS => '0');
    ELSIF RISING_EDGE(s_clk_500k) THEN
      s_mot_gauche_dir_reg <= s_dir_g;
      s_mot_droit_dir_reg  <= s_dir_d;
      s_led_reg            <= s_led7 & s_code_mode & s_sol_noir;
    END IF;
  END PROCESS registre_sorties;

  mot_gauche_dir <= s_mot_gauche_dir_reg;
  mot_droit_dir  <= s_mot_droit_dir_reg;
  led            <= s_led_reg;

END ARCHITECTURE robot_sumo_top_arch;
