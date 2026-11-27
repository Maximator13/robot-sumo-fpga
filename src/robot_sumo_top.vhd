--------------------------------------------------------------------------------
-- Fichier      : robot_sumo_top.vhd
-- Entité       : robot_sumo_top
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Niveau le plus haut du robot SUMO : relie tous les
--                sous-systèmes entre eux et aux broches de la carte. Le robot
--                a deux modes choisis depuis le PC par la liaison série :
--                'c' = combat SUMO, 's' = suiveur de ligne. À la mise sous
--                tension, il envoie "Robot PRET !" au PC (cahier des charges §1).
-- Principe     : Description structurelle (cours VHDL §7) : chaque
--                sous-système est déclaré par un COMPONENT, puis instancié et
--                relié aux autres par des signaux s_....
--                Deux horloges :
--                - 50 MHz (oscillateur) : reset, diviseur, IP de l'ADC ;
--                - 500 kHz (sortie du diviseur, CDC §6.2) : tout le reste.
--                Logique propre au top :
--                - aiguillage des moteurs : machine de combat en mode combat,
--                  sinon suiveur de ligne (qui arrête les moteurs quand il
--                  n'est pas actif) ;
--                - sens réel des moteurs : les machines à états disent
--                  « avance » ou « recule » ; DIR_AVANT_GAUCHE / DROIT donnent
--                  le niveau de la broche qui fait avancer chaque roue (les
--                  moteurs sont montés en miroir, à vérifier sur le robot) ;
--                - ligne série forcée au repos ('1') pendant le reset et les
--                  2 cycles suivants : l'émetteur UART_TX fourni n'a pas de
--                  reset, sans ce masque le PC recevrait un faux caractère ;
--                - LED : led(7) = batterie, led(6 DOWNTO 5) = mode ("00"
--                  attente, "01" combat, "10" suiveur), led(4 DOWNTO 0) =
--                  capteurs de sol voyant du noir (aide au réglage du seuil).
-- Entrées      : clock_50 (PIN_R8), bp1_b (BP1), ir_gauche_b / ir_droit_b
--                (capteurs d'obstacle), uart_rx (PC), adc_sdat (ADC).
-- Sorties      : uart_tx (PC), mot_gauche_dir / mot_gauche_en,
--                mot_droit_dir / mot_droit_en (pont en H), adc_cs_n /
--                adc_saddr / adc_sclk (ADC), led (8 LED).
-- Génériques   : durées en cycles, valeurs réelles par défaut (réduites par
--                le banc de test) : RESET_CYCLES_G (1 ms à 50 MHz),
--                ATTENTE_COMBAT_CYCLES_G (5 s), CLIGNO_DEMI_CYCLES_G (0,5 s),
--                FILTRE_BATT_CYCLES_G (1 s), BORDURE_MAINTIEN_CYCLES_G
--                (300 ms), FIN_LIGNE_CYCLES_G (200 ms), à 500 kHz.
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY robot_sumo_top IS
  GENERIC (
    RESET_CYCLES_G            : NATURAL := 50_000;
    ATTENTE_COMBAT_CYCLES_G   : NATURAL := 2_500_000;
    CLIGNO_DEMI_CYCLES_G      : NATURAL := 250_000;
    FILTRE_BATT_CYCLES_G      : NATURAL := 500_000;
    BORDURE_MAINTIEN_CYCLES_G : NATURAL := 150_000;
    FIN_LIGNE_CYCLES_G        : NATURAL := 100_000
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

  -- Durée d'un bit série à 9600 bauds avec l'horloge de 500 kHz :
  -- 500 000 / 9600 = 52 cycles (9615 bauds, écart de 0,16 %).
  CONSTANT UART_CLKS_PER_BIT : INTEGER := 52;

  -- Niveau de la broche de sens qui fait AVANCER chaque roue (CALIBRATION,
  -- à vérifier avec l'outil de test du robot).
  CONSTANT DIR_AVANT_GAUCHE : STD_LOGIC := '1';
  CONSTANT DIR_AVANT_DROIT  : STD_LOGIC := '1';

  -- Reset général et horloge système.
  SIGNAL s_rst_b     : STD_LOGIC;
  SIGNAL s_rst_adc   : STD_LOGIC;
  SIGNAL s_clk_500k  : STD_LOGIC;

  -- Mesures de l'ADC (voies 0 à 5 ; 6 et 7 non câblées).
  SIGNAL s_ch0, s_ch1, s_ch2, s_ch3, s_ch4, s_ch5 : STD_LOGIC_VECTOR(11 DOWNTO 0);

  -- Informations capteurs et batterie.
  SIGNAL s_sol_noir      : STD_LOGIC_VECTOR(4 DOWNTO 0);
  SIGNAL s_bordure       : STD_LOGIC;
  SIGNAL s_adversaire    : STD_LOGIC;
  SIGNAL s_code_batterie : UNSIGNED(11 DOWNTO 0);
  SIGNAL s_led7          : STD_LOGIC;
  SIGNAL s_batt_vide     : STD_LOGIC;

  -- Liaison série.
  SIGNAL s_rx_dv     : STD_LOGIC;
  SIGNAL s_rx_octet  : STD_LOGIC_VECTOR(7 DOWNTO 0);
  SIGNAL s_tx_dv     : STD_LOGIC;
  SIGNAL s_tx_octet  : STD_LOGIC_VECTOR(7 DOWNTO 0);
  SIGNAL s_tx_actif  : STD_LOGIC;
  SIGNAL s_tx_serie  : STD_LOGIC;
  SIGNAL s_tx_libre  : STD_LOGIC_VECTOR(1 DOWNTO 0);   -- masque TX après reset

  -- Mode choisi par le PC.
  SIGNAL s_mode_combat  : STD_LOGIC;
  SIGNAL s_mode_suiveur : STD_LOGIC;

  -- Commandes moteurs de chaque machine à états.
  SIGNAL s_combat_avance_g, s_combat_avance_d   : STD_LOGIC;
  SIGNAL s_combat_vitesse_g, s_combat_vitesse_d : UNSIGNED(6 DOWNTO 0);
  SIGNAL s_suiv_avance_g, s_suiv_avance_d       : STD_LOGIC;
  SIGNAL s_suiv_vitesse_g, s_suiv_vitesse_d     : UNSIGNED(6 DOWNTO 0);

  -- Commandes retenues après aiguillage.
  SIGNAL s_avance_g, s_avance_d   : STD_LOGIC;
  SIGNAL s_vitesse_g, s_vitesse_d : UNSIGNED(6 DOWNTO 0);

  -- Bascules des sorties sens moteurs et LED.
  SIGNAL s_mot_gauche_dir_reg : STD_LOGIC;
  SIGNAL s_mot_droit_dir_reg  : STD_LOGIC;
  SIGNAL s_led_reg            : STD_LOGIC_VECTOR(7 DOWNTO 0);

  ------------------------------------------------------------------------------
  -- Déclaration des composants (cours VHDL §7.1).
  ------------------------------------------------------------------------------
  COMPONENT circuit_reset IS
    GENERIC (RESET_CYCLES_G : NATURAL);
    PORT (clk_in : IN  STD_LOGIC;
          bp_in  : IN  STD_LOGIC;
          rst_b  : OUT STD_LOGIC);
  END COMPONENT circuit_reset;

  COMPONENT diviseur_frequence IS
    PORT (clk_in     : IN  STD_LOGIC;
          rst_in_b   : IN  STD_LOGIC;
          clk_500KHz : OUT STD_LOGIC);
  END COMPONENT diviseur_frequence;

  -- IP « ADC Controller for DE-series Boards » (catalogue IP de Quartus,
  -- comme au TP2), fichiers dans ip/adc_ctrl.
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

  COMPONENT detection_capteurs IS
    GENERIC (BORDURE_MAINTIEN_CYCLES_G : NATURAL);
    PORT (clk                  : IN  STD_LOGIC;
          rst_b                : IN  STD_LOGIC;
          ch_batterie          : IN  STD_LOGIC_VECTOR(11 DOWNTO 0);
          ch_sol_centre_gauche : IN  STD_LOGIC_VECTOR(11 DOWNTO 0);
          ch_sol_centre        : IN  STD_LOGIC_VECTOR(11 DOWNTO 0);
          ch_sol_centre_droit  : IN  STD_LOGIC_VECTOR(11 DOWNTO 0);
          ch_sol_gauche        : IN  STD_LOGIC_VECTOR(11 DOWNTO 0);
          ch_sol_droit         : IN  STD_LOGIC_VECTOR(11 DOWNTO 0);
          ir_gauche_b          : IN  STD_LOGIC;
          ir_droit_b           : IN  STD_LOGIC;
          sol_noir             : OUT STD_LOGIC_VECTOR(4 DOWNTO 0);
          bordure              : OUT STD_LOGIC;
          adversaire           : OUT STD_LOGIC;
          code_batterie        : OUT UNSIGNED(11 DOWNTO 0));
  END COMPONENT detection_capteurs;

  COMPONENT batterie_monitor IS
    GENERIC (CLIGNO_DEMI_CYCLES_G : NATURAL;
             FILTRE_BATT_CYCLES_G : NATURAL);
    PORT (clk           : IN  STD_LOGIC;
          rst_b         : IN  STD_LOGIC;
          code_batterie : IN  UNSIGNED(11 DOWNTO 0);
          led7          : OUT STD_LOGIC;
          batt_vide     : OUT STD_LOGIC);
  END COMPONENT batterie_monitor;

  -- Récepteur et émetteur série fournis (entités UART_RX et UART_TX).
  -- VHDL ne distingue pas majuscules et minuscules : un composant nommé
  -- UART_RX entrerait en conflit avec le port uart_rx. Les composants portent
  -- donc un autre nom et sont reliés aux entités fournies par une
  -- configuration (cours VHDL §8, lignes FOR ... USE ENTITY ci-dessous).
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

  FOR u_UART_RX : recepteur_serie USE ENTITY WORK.UART_RX;
  FOR u_UART_TX : emetteur_serie  USE ENTITY WORK.UART_TX;

  COMPONENT message_demarrage IS
    PORT (clk      : IN  STD_LOGIC;
          rst_b    : IN  STD_LOGIC;
          tx_actif : IN  STD_LOGIC;
          tx_dv    : OUT STD_LOGIC;
          tx_octet : OUT STD_LOGIC_VECTOR(7 DOWNTO 0));
  END COMPONENT message_demarrage;

  COMPONENT selection_mode IS
    PORT (clk          : IN  STD_LOGIC;
          rst_b        : IN  STD_LOGIC;
          rx_dv        : IN  STD_LOGIC;
          rx_octet     : IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
          mode_combat  : OUT STD_LOGIC;
          mode_suiveur : OUT STD_LOGIC);
  END COMPONENT selection_mode;

  COMPONENT fsm_combat IS
    GENERIC (ATTENTE_CYCLES_G : NATURAL);
    PORT (clk            : IN  STD_LOGIC;
          rst_b          : IN  STD_LOGIC;
          actif          : IN  STD_LOGIC;
          adversaire     : IN  STD_LOGIC;
          bordure        : IN  STD_LOGIC;
          batt_vide      : IN  STD_LOGIC;
          avance_gauche  : OUT STD_LOGIC;
          avance_droite  : OUT STD_LOGIC;
          vitesse_gauche : OUT UNSIGNED(6 DOWNTO 0);
          vitesse_droite : OUT UNSIGNED(6 DOWNTO 0));
  END COMPONENT fsm_combat;

  COMPONENT suiveur_ligne IS
    GENERIC (FIN_LIGNE_CYCLES_G : NATURAL);
    PORT (clk            : IN  STD_LOGIC;
          rst_b          : IN  STD_LOGIC;
          actif          : IN  STD_LOGIC;
          sol_noir       : IN  STD_LOGIC_VECTOR(4 DOWNTO 0);
          batt_vide      : IN  STD_LOGIC;
          avance_gauche  : OUT STD_LOGIC;
          avance_droite  : OUT STD_LOGIC;
          vitesse_gauche : OUT UNSIGNED(6 DOWNTO 0);
          vitesse_droite : OUT UNSIGNED(6 DOWNTO 0));
  END COMPONENT suiveur_ligne;

  COMPONENT pwm_gen IS
    PORT (clk   : IN  STD_LOGIC;
          rst_b : IN  STD_LOGIC;
          duty  : IN  UNSIGNED(6 DOWNTO 0);
          pwm   : OUT STD_LOGIC);
  END COMPONENT pwm_gen;

BEGIN

  ------------------------------------------------------------------------------
  -- Reset et horloge système (50 MHz)
  ------------------------------------------------------------------------------
  u_circuit_reset : circuit_reset
    GENERIC MAP (RESET_CYCLES_G => RESET_CYCLES_G)
    PORT MAP (clk_in => clock_50, bp_in => bp1_b, rst_b => s_rst_b);

  u_diviseur_frequence : diviseur_frequence
    PORT MAP (clk_in => clock_50, rst_in_b => s_rst_b, clk_500KHz => s_clk_500k);

  ------------------------------------------------------------------------------
  -- Acquisition : ADC (50 MHz) puis interprétation des capteurs (500 kHz)
  ------------------------------------------------------------------------------
  -- Le reset de l'IP est actif HAUT : on lui envoie l'inverse de rst_b.
  s_rst_adc <= NOT s_rst_b;

  u_adc_ctrl : adc_ctrl
    PORT MAP (CLOCK    => clock_50,
              RESET    => s_rst_adc,
              ADC_SCLK => adc_sclk,
              ADC_CS_N => adc_cs_n,
              ADC_DOUT => adc_sdat,     -- résultat série envoyé par l'ADC
              ADC_DIN  => adc_saddr,    -- adresse de la voie envoyée à l'ADC
              CH0 => s_ch0, CH1 => s_ch1, CH2 => s_ch2,
              CH3 => s_ch3, CH4 => s_ch4, CH5 => s_ch5,
              CH6 => OPEN, CH7 => OPEN);

  -- Câblage des voies (schéma de la carte d'interface) : 0 batterie,
  -- 1 centre gauche, 2 centre, 3 centre droit, 4 gauche, 5 droit.
  u_detection_capteurs : detection_capteurs
    GENERIC MAP (BORDURE_MAINTIEN_CYCLES_G => BORDURE_MAINTIEN_CYCLES_G)
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b,
              ch_batterie => s_ch0, ch_sol_centre_gauche => s_ch1,
              ch_sol_centre => s_ch2, ch_sol_centre_droit => s_ch3,
              ch_sol_gauche => s_ch4, ch_sol_droit => s_ch5,
              ir_gauche_b => ir_gauche_b, ir_droit_b => ir_droit_b,
              sol_noir => s_sol_noir, bordure => s_bordure,
              adversaire => s_adversaire, code_batterie => s_code_batterie);

  u_batterie_monitor : batterie_monitor
    GENERIC MAP (CLIGNO_DEMI_CYCLES_G => CLIGNO_DEMI_CYCLES_G,
                 FILTRE_BATT_CYCLES_G => FILTRE_BATT_CYCLES_G)
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b, code_batterie => s_code_batterie,
              led7 => s_led7, batt_vide => s_batt_vide);

  ------------------------------------------------------------------------------
  -- Liaison série avec le PC (9600 bauds, 8N1)
  ------------------------------------------------------------------------------
  u_UART_RX : recepteur_serie
    GENERIC MAP (g_CLKS_PER_BIT => UART_CLKS_PER_BIT)
    PORT MAP (i_Clk => s_clk_500k, i_RX_Serial => uart_rx,
              o_RX_DV => s_rx_dv, o_RX_Byte => s_rx_octet);

  u_UART_TX : emetteur_serie
    GENERIC MAP (g_CLKS_PER_BIT => UART_CLKS_PER_BIT)
    PORT MAP (i_Clk => s_clk_500k, i_TX_DV => s_tx_dv, i_TX_Byte => s_tx_octet,
              o_TX_Active => s_tx_actif, o_TX_Serial => s_tx_serie, o_TX_Done => OPEN);

  u_message_demarrage : message_demarrage
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b, tx_actif => s_tx_actif,
              tx_dv => s_tx_dv, tx_octet => s_tx_octet);

  -- Masque de la ligne TX : s_tx_libre vaut "00" pendant le reset, puis se
  -- remplit de '1' ; la ligne reste au repos ('1') tant qu'il n'est pas plein.
  validation_tx : PROCESS (s_clk_500k, s_rst_b)
  BEGIN
    IF s_rst_b = '0' THEN
      s_tx_libre <= "00";
    ELSIF RISING_EDGE(s_clk_500k) THEN
      s_tx_libre <= s_tx_libre(0) & '1';
    END IF;
  END PROCESS validation_tx;

  uart_tx <= s_tx_serie OR NOT s_tx_libre(1);

  u_selection_mode : selection_mode
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b, rx_dv => s_rx_dv, rx_octet => s_rx_octet,
              mode_combat => s_mode_combat, mode_suiveur => s_mode_suiveur);

  ------------------------------------------------------------------------------
  -- Comportements (machines à états)
  ------------------------------------------------------------------------------
  u_fsm_combat : fsm_combat
    GENERIC MAP (ATTENTE_CYCLES_G => ATTENTE_COMBAT_CYCLES_G)
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b, actif => s_mode_combat,
              adversaire => s_adversaire, bordure => s_bordure, batt_vide => s_batt_vide,
              avance_gauche => s_combat_avance_g, avance_droite => s_combat_avance_d,
              vitesse_gauche => s_combat_vitesse_g, vitesse_droite => s_combat_vitesse_d);

  u_suiveur_ligne : suiveur_ligne
    GENERIC MAP (FIN_LIGNE_CYCLES_G => FIN_LIGNE_CYCLES_G)
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b, actif => s_mode_suiveur,
              sol_noir => s_sol_noir, batt_vide => s_batt_vide,
              avance_gauche => s_suiv_avance_g, avance_droite => s_suiv_avance_d,
              vitesse_gauche => s_suiv_vitesse_g, vitesse_droite => s_suiv_vitesse_d);

  ------------------------------------------------------------------------------
  -- Aiguillage des commandes moteurs et PWM 5 kHz
  ------------------------------------------------------------------------------
  -- Combat en mode combat ; sinon suiveur de ligne (vitesse nulle s'il
  -- n'est pas actif, donc moteurs arrêtés en attente).
  s_avance_g  <= s_combat_avance_g  WHEN s_mode_combat = '1' ELSE s_suiv_avance_g;
  s_avance_d  <= s_combat_avance_d  WHEN s_mode_combat = '1' ELSE s_suiv_avance_d;
  s_vitesse_g <= s_combat_vitesse_g WHEN s_mode_combat = '1' ELSE s_suiv_vitesse_g;
  s_vitesse_d <= s_combat_vitesse_d WHEN s_mode_combat = '1' ELSE s_suiv_vitesse_d;

  -- PWM 5 kHz sur les entrées ENABLE du pont en H : '0' = moteur arrêté.
  u_pwm_gauche : pwm_gen
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b, duty => s_vitesse_g, pwm => mot_gauche_en);

  u_pwm_droit : pwm_gen
    PORT MAP (clk => s_clk_500k, rst_b => s_rst_b, duty => s_vitesse_d, pwm => mot_droit_en);

  ------------------------------------------------------------------------------
  -- Sorties sens moteurs et LED, recopiées dans des bascules
  ------------------------------------------------------------------------------
  registre_sorties : PROCESS (s_clk_500k, s_rst_b)
  BEGIN
    IF s_rst_b = '0' THEN
      s_mot_gauche_dir_reg <= DIR_AVANT_GAUCHE;
      s_mot_droit_dir_reg  <= DIR_AVANT_DROIT;
      s_led_reg            <= (OTHERS => '0');
    ELSIF RISING_EDGE(s_clk_500k) THEN
      -- Broche de sens : niveau "avant" si la roue avance, l'inverse sinon.
      IF s_avance_g = '1' THEN
        s_mot_gauche_dir_reg <= DIR_AVANT_GAUCHE;
      ELSE
        s_mot_gauche_dir_reg <= NOT DIR_AVANT_GAUCHE;
      END IF;
      IF s_avance_d = '1' THEN
        s_mot_droit_dir_reg <= DIR_AVANT_DROIT;
      ELSE
        s_mot_droit_dir_reg <= NOT DIR_AVANT_DROIT;
      END IF;
      -- LED : batterie, mode (suiveur, combat), capteurs de sol.
      s_led_reg <= s_led7 & s_mode_suiveur & s_mode_combat & s_sol_noir;
    END IF;
  END PROCESS registre_sorties;

  mot_gauche_dir <= s_mot_gauche_dir_reg;
  mot_droit_dir  <= s_mot_droit_dir_reg;
  led            <= s_led_reg;

END ARCHITECTURE robot_sumo_top_arch;
