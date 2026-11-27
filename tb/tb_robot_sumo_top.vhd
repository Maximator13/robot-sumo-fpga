--------------------------------------------------------------------------------
-- Fichier      : tb_robot_sumo_top.vhd
-- Entité       : tb_robot_sumo_top
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test SYSTÈME auto-vérifiant du robot complet
--                (robot_sumo_top), dans un environnement simulé : oscillateur
--                50 MHz, modèle de l'ADC128S022 (batterie + capteurs de sol),
--                capteurs d'obstacle, bouton BP1, et un « PC » relié à la
--                liaison série (réception et envoi à 9600 bauds 8N1).
-- Principe     : Les durées longues sont réduites par les génériques du top
--                (reset, attente 5 s, filtre batterie, clignotement, maintien
--                bordure, fin de ligne) ; la liaison série et la PWM gardent
--                leurs valeurs réelles. Scénario vérifié :
--                1) mise sous tension : ligne série au repos pendant le reset,
--                   moteurs arrêtés, puis message "Robot PRET !\n\r" reçu ;
--                2) 'c' : mode combat (LED), moteurs arrêtés pendant
--                   l'attente, puis rotation de recherche (gauche avant,
--                   droite arrière, 40 %) ;
--                3) adversaire vu à gauche seul : attaque, roue gauche à 25 % ;
--                   puis des deux côtés : attaque à 100 % en avant ;
--                4) un capteur de sol voit la bordure claire : recul à 70 % ;
--                5) batterie basse prolongée : moteurs arrêtés, LED7 clignote ;
--                6) appui sur BP1 : reset, nouveau message, mode attente ;
--                7) 's' : suiveur de ligne, ligne sous le capteur central :
--                   les deux roues avancent à 60 % ; sous le capteur centre
--                   gauche : roue gauche à 25 %, roue droite à 60 % ; sous
--                   le capteur gauche seul : pivot à gauche (roue gauche en
--                   arrière, droite en avant, 60 / 60) ; ligne recentrée puis
--                   perdue plus longtemps que le générique « fin de ligne » :
--                   extrémité, demi-tour en pivot à droite (roue droite en
--                   arrière) ; puis batterie basse prolongée pendant le
--                   demi-tour : moteurs arrêtés, LED7 clignote.
--                Les cas asymétriques (3 et 7, vitesses et sens) détectent une
--                inversion des moteurs gauche / droite.
--                Contrôles de structure : sens moteurs recopiés depuis des
--                bascules. Chien de garde à 80 ms. "TEST OK" seulement si
--                aucun échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY tb_robot_sumo_top IS
END ENTITY tb_robot_sumo_top;

ARCHITECTURE tb_robot_sumo_top_arch OF tb_robot_sumo_top IS

  CONSTANT PERIODE_50M : TIME := 20 ns;
  CONSTANT BIT_9600    : TIME := 104_167 ns;               -- bit série côté PC
  CONSTANT DUREE_BIT   : TIME := UART_CLKS_PER_BIT * 2 us; -- bit série côté robot (104 µs)
  CONSTANT MESSAGE_ATTENDU : STRING := "Robot PRET !" & LF & CR;

  -- Génériques réduits pour la simulation.
  CONSTANT RESET_SIM     : NATURAL := 200;    -- 4 µs à 50 MHz
  CONSTANT ATTENTE_SIM   : NATURAL := 500;    -- 1 ms à 500 kHz
  CONSTANT CLIGNO_SIM    : NATURAL := 100;    -- 200 µs
  CONSTANT FILTRE_SIM    : NATURAL := 300;    -- 600 µs
  CONSTANT BORDURE_SIM   : NATURAL := 100;    -- 200 µs
  CONSTANT FIN_LIGNE_SIM : NATURAL := 200;    -- 400 µs

  -- Délai entre un changement de tension vu par l'ADC et l'effet sur les
  -- moteurs, en plus du filtre batterie : l'IP de l'ADC ne publie ses 8
  -- voies qu'à la fin d'un tour complet (jusqu'à 2 tours, environ 90 µs),
  -- puis le générateur PWM n'applique la nouvelle consigne qu'au début de
  -- la période suivante (jusqu'à 200 µs à 5 kHz). Marge totale : 300 µs.
  CONSTANT LATENCE_ADC_PWM : TIME := 300 us;

  -- Codes ADC utiles.
  CONSTANT CODE_NOIR  : t_mot_adc := TO_UNSIGNED(3200, 12);  -- sol noir (> SEUIL_NOIR)
  CONSTANT CODE_CLAIR : t_mot_adc := TO_UNSIGNED(400, 12);   -- sol clair (bordure, piste)
  CONSTANT CODE_BAT_OK  : t_mot_adc := TO_UNSIGNED(3200, 12); -- environ 8,4 V
  CONSTANT CODE_BAT_BAS : t_mot_adc := TO_UNSIGNED(2400, 12); -- environ 6,5 V (< 7,2 V)

  SIGNAL clock_50    : STD_LOGIC := '0';
  SIGNAL bp1_b       : STD_LOGIC := '1';
  SIGNAL ir_gauche_b : STD_LOGIC := '1';
  SIGNAL ir_droit_b  : STD_LOGIC := '1';
  SIGNAL uart_rx     : STD_LOGIC := '1';   -- ligne PC -> robot
  SIGNAL uart_tx     : STD_LOGIC;          -- ligne robot -> PC
  SIGNAL mot_gauche_dir, mot_gauche_en, mot_droit_dir, mot_droit_en : STD_LOGIC;
  SIGNAL adc_cs_n, adc_saddr, adc_sclk, adc_sdat : STD_LOGIC;
  SIGNAL led         : STD_LOGIC_VECTOR(7 DOWNTO 0);
  SIGNAL fin_simu    : BOOLEAN := FALSE;

  -- Valeurs analogiques vues par l'ADC : batterie et 5 capteurs de sol.
  SIGNAL valeurs     : t_voies_adc := (OTHERS => (OTHERS => '0'));
  SIGNAL nb_conv, err_timing, incompletes : NATURAL;

  -- Journal du PC (octets reçus).
  TYPE t_octets IS ARRAY (0 TO 63) OF STD_LOGIC_VECTOR(7 DOWNTO 0);
  SIGNAL octets_recus : t_octets;
  SIGNAL nb_recus     : NATURAL := 0;

  SIGNAL erreurs_struct : NATURAL := 0;

  FUNCTION ascii (c : CHARACTER) RETURN STD_LOGIC_VECTOR IS
  BEGIN
    RETURN STD_LOGIC_VECTOR(TO_UNSIGNED(CHARACTER'POS(c), 8));
  END FUNCTION ascii;

BEGIN

  u_robot_sumo_top : ENTITY WORK.robot_sumo_top
    GENERIC MAP (
      RESET_CYCLES_G            => RESET_SIM,
      ATTENTE_COMBAT_CYCLES_G   => ATTENTE_SIM,
      CLIGNO_DEMI_CYCLES_G      => CLIGNO_SIM,
      FILTRE_BATT_CYCLES_G      => FILTRE_SIM,
      BORDURE_MAINTIEN_CYCLES_G => BORDURE_SIM,
      FIN_LIGNE_CYCLES_G        => FIN_LIGNE_SIM)
    PORT MAP (
      clock_50 => clock_50, bp1_b => bp1_b, ir_gauche_b => ir_gauche_b, ir_droit_b => ir_droit_b,
      uart_rx => uart_rx, uart_tx => uart_tx,
      mot_gauche_dir => mot_gauche_dir, mot_gauche_en => mot_gauche_en,
      mot_droit_dir => mot_droit_dir, mot_droit_en => mot_droit_en,
      adc_cs_n => adc_cs_n, adc_saddr => adc_saddr, adc_sclk => adc_sclk, adc_sdat => adc_sdat,
      led => led);

  u_modele_adc : ENTITY WORK.adc128s022_model
    PORT MAP (cs_n => adc_cs_n, sclk => adc_sclk, din => adc_saddr, dout => adc_sdat,
              valeurs => valeurs, nb_conversions => nb_conv, erreurs_timing => err_timing,
              trames_incompletes => incompletes);

  horloge : PROCESS
  BEGIN
    WHILE NOT fin_simu LOOP
      clock_50 <= '0'; WAIT FOR PERIODE_50M / 2;
      clock_50 <= '1'; WAIT FOR PERIODE_50M / 2;
    END LOOP;
    WAIT;
  END PROCESS horloge;

  -- Le PC décode chaque trame reçue sur uart_tx.
  pc_reception : PROCESS
    VARIABLE v : STD_LOGIC_VECTOR(7 DOWNTO 0);
  BEGIN
    WAIT UNTIL FALLING_EDGE(uart_tx);
    WAIT FOR DUREE_BIT / 2;
    IF uart_tx = '0' THEN
      FOR i IN 0 TO 7 LOOP
        WAIT FOR DUREE_BIT;
        v(i) := uart_tx;
      END LOOP;
      WAIT FOR DUREE_BIT;
      IF uart_tx = '1' AND nb_recus < 64 THEN
        octets_recus(nb_recus) <= v;
        nb_recus <= nb_recus + 1;
      END IF;
    END IF;
  END PROCESS pc_reception;

  -- Contrôle de structure : les sens moteurs sortent de bascules du top.
  sorties_registrees : PROCESS
    ALIAS dir_g_reg IS << SIGNAL .tb_robot_sumo_top.u_robot_sumo_top.s_mot_gauche_dir_reg : STD_LOGIC >>;
    ALIAS dir_d_reg IS << SIGNAL .tb_robot_sumo_top.u_robot_sumo_top.s_mot_droit_dir_reg  : STD_LOGIC >>;
  BEGIN
    WAIT ON clock_50;
    WAIT FOR 1 ns;
    IF mot_gauche_dir /= dir_g_reg OR mot_droit_dir /= dir_d_reg THEN
      erreurs_struct <= erreurs_struct + 1;
      REPORT "ECHEC : sens moteur pas recopié depuis sa bascule" SEVERITY ERROR;
    END IF;
  END PROCESS sorties_registrees;

  chien_de_garde : PROCESS
  BEGIN
    WAIT FOR 80 ms;
    IF NOT fin_simu THEN
      REPORT "TEST KO : tb_robot_sumo_top bloqué (délai de 80 ms dépassé)" SEVERITY FAILURE;
    END IF;
    WAIT;
  END PROCESS chien_de_garde;

  scenario : PROCESS
    ALIAS clk_sys IS << SIGNAL .tb_robot_sumo_top.u_robot_sumo_top.s_clk_500k : STD_LOGIC >>;
    ALIAS rst_sys IS << SIGNAL .tb_robot_sumo_top.u_robot_sumo_top.s_rst_b    : STD_LOGIC >>;
    VARIABLE nb_controles : NATURAL := 0;
    VARIABLE nb_echecs    : NATURAL := 0;
    VARIABLE hauts_g, hauts_d : NATURAL;
    VARIABLE premier : NATURAL;
    -- Violations de timing SPI dues à la trame interrompue par l'appui sur
    -- BP1 (voir le scénario 6).
    VARIABLE err_avant_bp1, err_trame_bp1 : NATURAL;

    PROCEDURE verifier (condition : BOOLEAN; message : STRING) IS
    BEGIN
      nb_controles := nb_controles + 1;
      IF NOT condition THEN
        nb_echecs := nb_echecs + 1;
      END IF;
      ASSERT condition REPORT "ECHEC : " & message SEVERITY ERROR;
    END PROCEDURE verifier;

    -- Le PC envoie un caractère à 9600 bauds.
    PROCEDURE pc_envoie (c : CHARACTER) IS
      VARIABLE v : STD_LOGIC_VECTOR(7 DOWNTO 0);
    BEGIN
      v := ascii(c);
      uart_rx <= '0'; WAIT FOR BIT_9600;
      FOR i IN 0 TO 7 LOOP
        uart_rx <= v(i); WAIT FOR BIT_9600;
      END LOOP;
      uart_rx <= '1'; WAIT FOR BIT_9600;
    END PROCEDURE pc_envoie;

    -- Mesure les rapports cycliques des deux PWM moteurs sur une période PWM
    -- complète (100 cycles de l'horloge système 500 kHz).
    PROCEDURE mesurer_pwm (dg, dd : OUT NATURAL) IS
      VARIABLE g, d : NATURAL := 0;
    BEGIN
      FOR i IN 1 TO PWM_PERIODE_CYCLES LOOP
        WAIT UNTIL RISING_EDGE(clk_sys);
        WAIT FOR 100 ns;
        IF mot_gauche_en = '1' THEN g := g + 1; END IF;
        IF mot_droit_en  = '1' THEN d := d + 1; END IF;
      END LOOP;
      dg := g; dd := d;
    END PROCEDURE mesurer_pwm;

    -- Attend la fin d'un message de démarrage puis vérifie les 14 derniers octets.
    PROCEDURE verifier_message (contexte : STRING) IS
    BEGIN
      WAIT FOR 20 ms;
      premier := nb_recus - MESSAGE_ATTENDU'LENGTH;
      verifier(nb_recus >= MESSAGE_ATTENDU'LENGTH, contexte & " : message incomplet (" & INTEGER'IMAGE(nb_recus) & " octets)");
      IF nb_recus >= MESSAGE_ATTENDU'LENGTH THEN
        FOR i IN 0 TO MESSAGE_ATTENDU'LENGTH - 1 LOOP
          verifier(octets_recus(premier + i) = ascii(MESSAGE_ATTENDU(i + 1)),
                   contexte & " : caractère " & INTEGER'IMAGE(i) & " du message incorrect");
        END LOOP;
      END IF;
    END PROCEDURE verifier_message;

  BEGIN
    -- Dohyo : sol noir partout, batterie chargée.
    valeurs(VOIE_BAT)               <= CODE_BAT_OK;
    valeurs(VOIE_SOL_GAUCHE)        <= CODE_NOIR;
    valeurs(VOIE_SOL_CENTRE_GAUCHE) <= CODE_NOIR;
    valeurs(VOIE_SOL_CENTRE)        <= CODE_NOIR;
    valeurs(VOIE_SOL_CENTRE_DROIT)  <= CODE_NOIR;
    valeurs(VOIE_SOL_DROIT)         <= CODE_NOIR;

    ----------------------------------------------------------------------------
    -- 1. Mise sous tension : ligne série au repos pendant le reset, moteurs
    --    arrêtés, puis message de démarrage complet.
    ----------------------------------------------------------------------------
    WAIT FOR 1 ns;
    WHILE rst_sys /= '1' LOOP
      verifier(uart_tx = '1', "ligne série pas au repos pendant le reset");
      verifier(mot_gauche_en = '0' AND mot_droit_en = '0', "moteurs actifs pendant le reset");
      WAIT FOR 200 ns;
    END LOOP;
    verifier_message("mise sous tension");
    verifier(nb_recus = MESSAGE_ATTENDU'LENGTH, "octets en trop au démarrage (" & INTEGER'IMAGE(nb_recus) & ")");
    verifier(led(6 DOWNTO 5) = "00", "mode attente non affiché");
    mesurer_pwm(hauts_g, hauts_d);
    verifier(hauts_g = 0 AND hauts_d = 0, "moteurs actifs en attente de commande");
    verifier(led(4 DOWNTO 0) = "11111", "LED capteurs : sol noir partout attendu");

    ----------------------------------------------------------------------------
    -- 2. 'c' : mode combat, attente puis rotation de recherche.
    ----------------------------------------------------------------------------
    pc_envoie('c');
    WAIT FOR 20 us;
    verifier(led(6 DOWNTO 5) = "01", "mode combat non affiché après 'c'");
    mesurer_pwm(hauts_g, hauts_d);
    verifier(hauts_g = 0 AND hauts_d = 0, "moteurs actifs pendant l'attente avant combat");
    WAIT FOR ATTENTE_SIM * 2 us;
    mesurer_pwm(hauts_g, hauts_d);
    mesurer_pwm(hauts_g, hauts_d);
    verifier(hauts_g = VITESSE_RECHERCHE AND hauts_d = VITESSE_RECHERCHE,
             "recherche : PWM " & INTEGER'IMAGE(hauts_g) & " / " & INTEGER'IMAGE(hauts_d) & " au lieu de 40 / 40");
    verifier(mot_gauche_dir = DIR_AVANT_GAUCHE AND mot_droit_dir = NOT DIR_AVANT_DROIT,
             "recherche : rotation attendue (gauche avant, droite arrière)");

    ----------------------------------------------------------------------------
    -- 3. Adversaire vu à gauche seulement : attaque, roue gauche ralentie
    --    (cas asymétrique : vérifie aussi que les moteurs ne sont pas
    --    inversés). Puis vu des deux côtés : pleine vitesse en avant.
    ----------------------------------------------------------------------------
    ir_gauche_b <= '0';
    WAIT FOR 500 us;
    mesurer_pwm(hauts_g, hauts_d);
    verifier(hauts_g = VITESSE_VIRAGE AND hauts_d = VITESSE_ATTAQUE,
             "attaque à gauche : PWM " & INTEGER'IMAGE(hauts_g) & " / " & INTEGER'IMAGE(hauts_d) & " au lieu de 25 / 100");
    verifier(mot_gauche_dir = DIR_AVANT_GAUCHE AND mot_droit_dir = DIR_AVANT_DROIT, "attaque à gauche : les deux roues doivent avancer");
    ir_droit_b <= '0';
    WAIT FOR 500 us;
    mesurer_pwm(hauts_g, hauts_d);
    verifier(hauts_g = VITESSE_ATTAQUE AND hauts_d = VITESSE_ATTAQUE,
             "attaque : PWM " & INTEGER'IMAGE(hauts_g) & " / " & INTEGER'IMAGE(hauts_d) & " au lieu de 100 / 100");
    verifier(mot_gauche_dir = DIR_AVANT_GAUCHE AND mot_droit_dir = DIR_AVANT_DROIT, "attaque : les deux roues doivent avancer");

    ----------------------------------------------------------------------------
    -- 4. Bordure claire sous le capteur gauche : recul.
    ----------------------------------------------------------------------------
    valeurs(VOIE_SOL_GAUCHE) <= CODE_CLAIR;
    WAIT FOR 500 us;
    mesurer_pwm(hauts_g, hauts_d);
    verifier(hauts_g = VITESSE_RECUL AND hauts_d = VITESSE_RECUL,
             "bordure : PWM " & INTEGER'IMAGE(hauts_g) & " / " & INTEGER'IMAGE(hauts_d) & " au lieu de 70 / 70");
    verifier(mot_gauche_dir = NOT DIR_AVANT_GAUCHE AND mot_droit_dir = NOT DIR_AVANT_DROIT, "bordure : les deux roues doivent reculer");
    verifier(led(4) = '0', "LED du capteur gauche : sol clair attendu");
    valeurs(VOIE_SOL_GAUCHE) <= CODE_NOIR;
    ir_gauche_b <= '1'; ir_droit_b <= '1';

    ----------------------------------------------------------------------------
    -- 5. Batterie basse prolongée : arrêt définitif, LED7 clignote.
    ----------------------------------------------------------------------------
    valeurs(VOIE_BAT) <= CODE_BAT_BAS;
    WAIT FOR (FILTRE_SIM + 50) * 2 us + LATENCE_ADC_PWM;
    mesurer_pwm(hauts_g, hauts_d);
    verifier(hauts_g = 0 AND hauts_d = 0, "batterie vide : moteurs pas arrêtés");
    WAIT UNTIL led(7) = '1' FOR 2 ms;
    WAIT UNTIL led(7) = '0' FOR 2 ms;
    verifier(led(7) = '0', "batterie vide : LED7 ne clignote pas");
    WAIT UNTIL led(7) = '1' FOR 2 ms;
    verifier(led(7) = '1', "batterie vide : LED7 ne clignote pas");
    valeurs(VOIE_BAT) <= CODE_BAT_OK;
    WAIT FOR 1 ms;
    mesurer_pwm(hauts_g, hauts_d);
    verifier(hauts_g = 0 AND hauts_d = 0, "batterie vide : l'arrêt doit rester définitif jusqu'au reset");

    ----------------------------------------------------------------------------
    -- 6. Appui sur BP1 : reset, nouveau message, mode attente.
    ----------------------------------------------------------------------------
    -- L'IP de l'ADC (catalogue Quartus) ne remet pas sa machine à états à
    -- zéro pendant son reset : une trame SPI en cours reste figée (CS à '0',
    -- SCLK arrêtée) pendant le reset, puis se termine. Le modèle d'ADC voit
    -- donc des durées de SCLK hors datasheet sur cette seule trame. On compte
    -- à part les violations de cette trame (au plus 3 : période, tCL, tCH)
    -- et on exige toujours 0 violation partout ailleurs.
    err_avant_bp1 := err_timing;
    bp1_b <= '0'; WAIT FOR 10 us; bp1_b <= '1';
    WAIT FOR 100 ns;
    verifier(rst_sys = '0', "BP1 n'a pas déclenché le reset");
    verifier_message("après BP1");
    err_trame_bp1 := err_timing - err_avant_bp1;
    verifier(err_trame_bp1 <= 3, INTEGER'IMAGE(err_trame_bp1) & " violation(s) SPI autour de BP1 : plus d'une trame touchée");
    verifier(led(6 DOWNTO 5) = "00", "le reset ne remet pas le mode en attente");
    mesurer_pwm(hauts_g, hauts_d);
    verifier(hauts_g = 0 AND hauts_d = 0, "moteurs actifs après le reset");

    ----------------------------------------------------------------------------
    -- 7. 's' : suiveur de ligne, ligne noire sous le capteur central seul.
    ----------------------------------------------------------------------------
    valeurs(VOIE_SOL_GAUCHE)        <= CODE_CLAIR;
    valeurs(VOIE_SOL_CENTRE_GAUCHE) <= CODE_CLAIR;
    valeurs(VOIE_SOL_CENTRE)        <= CODE_NOIR;
    valeurs(VOIE_SOL_CENTRE_DROIT)  <= CODE_CLAIR;
    valeurs(VOIE_SOL_DROIT)         <= CODE_CLAIR;
    pc_envoie('s');
    WAIT FOR 500 us;
    verifier(led(6 DOWNTO 5) = "10", "mode suiveur non affiché après 's'");
    verifier(led(4 DOWNTO 0) = "00100", "LED capteurs : seul le capteur central sur la ligne");
    mesurer_pwm(hauts_g, hauts_d);
    mesurer_pwm(hauts_g, hauts_d);
    verifier(hauts_g = VITESSE_SUIVI AND hauts_d = VITESSE_SUIVI,
             "suivi : PWM " & INTEGER'IMAGE(hauts_g) & " / " & INTEGER'IMAGE(hauts_d) & " au lieu de 60 / 60");
    verifier(mot_gauche_dir = DIR_AVANT_GAUCHE AND mot_droit_dir = DIR_AVANT_DROIT, "suivi : les deux roues doivent avancer");
    -- Ligne sous le capteur centre gauche seul : virage doux à gauche (roue
    -- gauche ralentie), cas asymétrique.
    valeurs(VOIE_SOL_CENTRE)       <= CODE_CLAIR;
    valeurs(VOIE_SOL_CENTRE_GAUCHE) <= CODE_NOIR;
    WAIT FOR 500 us;
    verifier(led(4 DOWNTO 0) = "01000", "LED capteurs : seul le capteur centre gauche sur la ligne");
    mesurer_pwm(hauts_g, hauts_d);
    mesurer_pwm(hauts_g, hauts_d);
    verifier(hauts_g = VITESSE_VIRAGE AND hauts_d = VITESSE_SUIVI,
             "virage à gauche : PWM " & INTEGER'IMAGE(hauts_g) & " / " & INTEGER'IMAGE(hauts_d) & " au lieu de 25 / 60");
    -- Ligne sous le capteur gauche seul : pivot à gauche sur place. Seul cas
    -- du suiveur où les sens des deux roues diffèrent : détecte une
    -- inversion des sens gauche / droite dans l'aiguillage du top.
    valeurs(VOIE_SOL_GAUCHE)        <= CODE_NOIR;
    WAIT FOR 100 us;
    valeurs(VOIE_SOL_CENTRE_GAUCHE) <= CODE_CLAIR;
    WAIT FOR 500 us;
    verifier(led(4 DOWNTO 0) = "10000", "LED capteurs : seul le capteur gauche sur la ligne");
    mesurer_pwm(hauts_g, hauts_d);
    mesurer_pwm(hauts_g, hauts_d);
    verifier(hauts_g = VITESSE_SUIVI AND hauts_d = VITESSE_SUIVI,
             "pivot à gauche : PWM " & INTEGER'IMAGE(hauts_g) & " / " & INTEGER'IMAGE(hauts_d) & " au lieu de 60 / 60");
    verifier(mot_gauche_dir = NOT DIR_AVANT_GAUCHE AND mot_droit_dir = DIR_AVANT_DROIT,
             "pivot à gauche : roue gauche en arrière et roue droite en avant attendues");
    -- Ligne recentrée (le capteur central la voit avant que le gauche la
    -- perde, pour ne pas passer par une perte de ligne).
    valeurs(VOIE_SOL_CENTRE) <= CODE_NOIR;
    WAIT FOR 100 us;
    valeurs(VOIE_SOL_GAUCHE) <= CODE_CLAIR;
    WAIT FOR 500 us;
    verifier(led(4 DOWNTO 0) = "00100", "LED capteurs : ligne recentrée sous le capteur central");
    -- Extrémité de ligne : tout blanc après une ligne centrée, plus longtemps
    -- que FIN_LIGNE_SIM. Le suiveur doit faire demi-tour en pivotant à
    -- droite (roue gauche en avant, roue droite en arrière). Vérifie aussi
    -- que le générique réduit de fin de ligne est bien transmis au suiveur.
    valeurs(VOIE_SOL_CENTRE) <= CODE_CLAIR;
    WAIT FOR (FIN_LIGNE_SIM + 50) * 2 us;
    verifier(led(4 DOWNTO 0) = "00000", "LED capteurs : tout blanc attendu en fin de ligne");
    mesurer_pwm(hauts_g, hauts_d);
    mesurer_pwm(hauts_g, hauts_d);
    verifier(hauts_g = VITESSE_SUIVI AND hauts_d = VITESSE_SUIVI,
             "demi-tour : PWM " & INTEGER'IMAGE(hauts_g) & " / " & INTEGER'IMAGE(hauts_d) & " au lieu de 60 / 60");
    verifier(mot_gauche_dir = DIR_AVANT_GAUCHE AND mot_droit_dir = NOT DIR_AVANT_DROIT,
             "demi-tour : roue gauche en avant et roue droite en arrière attendues");
    -- Batterie basse prolongée pendant le demi-tour : le suiveur doit aussi
    -- arrêter les moteurs (CDC §6.4, quel que soit le mode), LED7 clignote.
    valeurs(VOIE_BAT) <= CODE_BAT_BAS;
    WAIT FOR (FILTRE_SIM + 50) * 2 us + LATENCE_ADC_PWM;
    mesurer_pwm(hauts_g, hauts_d);
    verifier(hauts_g = 0 AND hauts_d = 0, "batterie vide en mode suiveur : moteurs pas arrêtés");
    WAIT UNTIL led(7) = '1' FOR 2 ms;
    WAIT UNTIL led(7) = '0' FOR 2 ms;
    verifier(led(7) = '0', "batterie vide en mode suiveur : LED7 ne clignote pas");
    WAIT UNTIL led(7) = '1' FOR 2 ms;
    verifier(led(7) = '1', "batterie vide en mode suiveur : LED7 ne clignote pas");

    -- Aucune violation de timing vue par le modèle d'ADC.
    verifier(err_timing - err_trame_bp1 = 0, INTEGER'IMAGE(err_timing - err_trame_bp1) & " violation(s) de timing sur le bus de l'ADC (hors trame coupée par BP1)");

    -- Bilan.
    IF nb_echecs = 0 AND erreurs_struct = 0 THEN
      REPORT "TEST OK : tb_robot_sumo_top (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_robot_sumo_top (" & INTEGER'IMAGE(nb_echecs + erreurs_struct) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    std.env.finish;
  END PROCESS scenario;

END ARCHITECTURE tb_robot_sumo_top_arch;
