--------------------------------------------------------------------------------
-- Fichier      : tb_robot_sumo_top.vhd
-- Entité       : tb_robot_sumo_top
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant du robot complet.
-- Principe     : Horloge 50 MHz, durées réduites (reset 4 µs, attente du
--                combat 1 ms, filtre batterie 600 µs...). Le banc contient :
--                - le modèle de l'ADC128S022, relié à l'IP de l'ADC, qui
--                  vérifie les timings du bus SPI ;
--                - un "PC" qui lit les caractères envoyés par le robot et
--                  peut lui envoyer 'c' ou 's' ;
--                - la mesure du rapport cyclique PWM de chaque moteur.
--                Scénario : message "Robot PRET !" au démarrage, combat
--                (attente, recherche, attaque, recul au bord), batterie vide,
--                reset par BP1 (nouveau message), suivi de ligne.
--                "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY tb_robot_sumo_top IS
END ENTITY tb_robot_sumo_top;

ARCHITECTURE tb_robot_sumo_top_arch OF tb_robot_sumo_top IS

  -- Durées réduites pour la simulation (en cycles).
  CONSTANT RESET_SIM     : NATURAL := 200;   -- 4 µs à 50 MHz
  CONSTANT ATTENTE_SIM   : NATURAL := 500;   -- 1 ms à 500 kHz
  CONSTANT CLIGNO_SIM    : NATURAL := 100;   -- 200 µs
  CONSTANT FILTRE_SIM    : NATURAL := 300;   -- 600 µs
  CONSTANT BORDURE_SIM   : NATURAL := 100;   -- 200 µs
  CONSTANT FIN_LIGNE_SIM : NATURAL := 200;   -- 400 µs

  CONSTANT DUREE_BIT : TIME := 104 us;       -- un bit série (52 cycles de 2 µs)
  CONSTANT ATTENDU   : STRING := "Robot PRET !" & CHARACTER'VAL(10) & CHARACTER'VAL(13);

  -- Codes ADC : sol noir, sol clair, batterie chargée (8,4 V) ou vide (6,5 V).
  CONSTANT NOIR    : NATURAL := 3200;
  CONSTANT CLAIR   : NATURAL := 400;
  CONSTANT BAT_OK  : NATURAL := 3200;
  CONSTANT BAT_BAS : NATURAL := 2400;

  SIGNAL clock_50    : STD_LOGIC := '0';
  SIGNAL bp1_b       : STD_LOGIC := '1';
  SIGNAL ir_gauche_b : STD_LOGIC := '1';
  SIGNAL ir_droit_b  : STD_LOGIC := '1';
  SIGNAL uart_rx     : STD_LOGIC := '1';
  SIGNAL uart_tx     : STD_LOGIC;
  SIGNAL mot_g_dir, mot_g_en, mot_d_dir, mot_d_en : STD_LOGIC;
  SIGNAL adc_cs_n, adc_saddr, adc_sclk, adc_sdat  : STD_LOGIC;
  SIGNAL led         : STD_LOGIC_VECTOR(7 DOWNTO 0);
  SIGNAL fin_simu    : BOOLEAN := FALSE;

  -- Valeurs vues par l'ADC : voie k sur les bits 12k+11 à 12k.
  SIGNAL valeurs     : STD_LOGIC_VECTOR(95 DOWNTO 0) := (OTHERS => '0');
  SIGNAL err_timing  : NATURAL;

  -- Caractères reçus par le "PC".
  SIGNAL recu        : STRING(1 TO 40) := (OTHERS => ' ');
  SIGNAL nb_recus    : NATURAL := 0;

BEGIN

  u_robot : ENTITY WORK.robot_sumo_top
    GENERIC MAP (RESET_CYCLES_G => RESET_SIM, ATTENTE_COMBAT_CYCLES_G => ATTENTE_SIM,
                 CLIGNO_DEMI_CYCLES_G => CLIGNO_SIM, FILTRE_BATT_CYCLES_G => FILTRE_SIM,
                 BORDURE_MAINTIEN_CYCLES_G => BORDURE_SIM, FIN_LIGNE_CYCLES_G => FIN_LIGNE_SIM)
    PORT MAP (clock_50 => clock_50, bp1_b => bp1_b, ir_gauche_b => ir_gauche_b,
              ir_droit_b => ir_droit_b, uart_rx => uart_rx, uart_tx => uart_tx,
              mot_gauche_dir => mot_g_dir, mot_gauche_en => mot_g_en,
              mot_droit_dir => mot_d_dir, mot_droit_en => mot_d_en,
              adc_cs_n => adc_cs_n, adc_saddr => adc_saddr, adc_sclk => adc_sclk,
              adc_sdat => adc_sdat, led => led);

  u_modele_adc : ENTITY WORK.adc128s022_model
    PORT MAP (cs_n => adc_cs_n, sclk => adc_sclk, din => adc_saddr, dout => adc_sdat,
              valeurs => valeurs, nb_conversions => OPEN, erreurs_timing => err_timing,
              trames_incompletes => OPEN);

  horloge : PROCESS
  BEGIN
    WHILE NOT fin_simu LOOP
      clock_50 <= '0'; WAIT FOR 10 ns;
      clock_50 <= '1'; WAIT FOR 10 ns;
    END LOOP;
    WAIT;
  END PROCESS horloge;

  -- Le "PC" lit la ligne série du robot (bit lu en son milieu).
  pc_lecture : PROCESS
    VARIABLE octet : STD_LOGIC_VECTOR(7 DOWNTO 0);
  BEGIN
    WAIT UNTIL uart_tx = '1';
    LOOP
      WAIT UNTIL FALLING_EDGE(uart_tx);
      WAIT FOR DUREE_BIT / 2;
      FOR i IN 0 TO 7 LOOP
        WAIT FOR DUREE_BIT;
        octet(i) := uart_tx;
      END LOOP;
      WAIT FOR DUREE_BIT;
      IF nb_recus < 40 THEN
        recu(nb_recus + 1) <= CHARACTER'VAL(TO_INTEGER(UNSIGNED(octet)));
        nb_recus <= nb_recus + 1;
      END IF;
    END LOOP;
  END PROCESS pc_lecture;

  scenario : PROCESS
    VARIABLE nb_controles : NATURAL := 0;
    VARIABLE nb_echecs    : NATURAL := 0;
    VARIABLE pwm_g, pwm_d : NATURAL;
    VARIABLE err_avant_bp1, err_bp1 : NATURAL;

    PROCEDURE verifier (condition : BOOLEAN; message : STRING) IS
    BEGIN
      nb_controles := nb_controles + 1;
      IF NOT condition THEN
        nb_echecs := nb_echecs + 1;
        REPORT "ECHEC : " & message SEVERITY ERROR;
      END IF;
    END PROCEDURE verifier;

    -- Fixe la valeur vue par l'ADC sur une voie.
    PROCEDURE voie (k : NATURAL; code : NATURAL) IS
    BEGIN
      valeurs(12 * k + 11 DOWNTO 12 * k) <= STD_LOGIC_VECTOR(TO_UNSIGNED(code, 12));
    END PROCEDURE voie;

    -- Le PC envoie un caractère (start, 8 bits, stop).
    PROCEDURE pc_envoie (c : CHARACTER) IS
      VARIABLE v : STD_LOGIC_VECTOR(7 DOWNTO 0);
    BEGIN
      v := STD_LOGIC_VECTOR(TO_UNSIGNED(CHARACTER'POS(c), 8));
      uart_rx <= '0'; WAIT FOR DUREE_BIT;
      FOR i IN 0 TO 7 LOOP
        uart_rx <= v(i); WAIT FOR DUREE_BIT;
      END LOOP;
      uart_rx <= '1'; WAIT FOR DUREE_BIT;
    END PROCEDURE pc_envoie;

    -- Rapport cyclique des deux moteurs (en %) sur une période de 200 µs,
    -- échantillonnée toutes les 2 µs.
    PROCEDURE mesurer_pwm IS
    BEGIN
      pwm_g := 0;
      pwm_d := 0;
      WAIT FOR 0.3 us;
      FOR i IN 1 TO 100 LOOP
        IF mot_g_en = '1' THEN pwm_g := pwm_g + 1; END IF;
        IF mot_d_en = '1' THEN pwm_d := pwm_d + 1; END IF;
        WAIT FOR 2 us;
      END LOOP;
    END PROCEDURE mesurer_pwm;

    -- Vérifie le sens ('1' = broche à '1') et la vitesse des deux moteurs.
    PROCEDURE moteurs (dg, dd : STD_LOGIC; vg, vd : NATURAL; situation : STRING) IS
    BEGIN
      mesurer_pwm;
      verifier(pwm_g = vg AND pwm_d = vd AND mot_g_dir = dg AND mot_d_dir = dd,
               situation & " : PWM " & INTEGER'IMAGE(pwm_g) & "/" & INTEGER'IMAGE(pwm_d)
               & ", sens " & STD_LOGIC'IMAGE(mot_g_dir) & "/" & STD_LOGIC'IMAGE(mot_d_dir));
    END PROCEDURE moteurs;

  BEGIN
    -- Dohyo : sol noir sous les 5 capteurs, batterie chargée.
    voie(0, BAT_OK);
    FOR k IN 1 TO 5 LOOP
      voie(k, NOIR);
    END LOOP;

    ----------------------------------------------------------------------------
    -- 1. Mise sous tension : ligne série au repos, message "Robot PRET !".
    ----------------------------------------------------------------------------
    WAIT FOR 2 us;
    verifier(uart_tx = '1', "ligne série active pendant le reset");
    WAIT FOR 16 ms;
    verifier(nb_recus = 14 AND recu(1 TO 14) = ATTENDU, "message de démarrage incorrect");
    verifier(led(6 DOWNTO 5) = "00", "un mode est actif au démarrage");
    moteurs('1', '1', 0, 0, "démarrage");

    ----------------------------------------------------------------------------
    -- 2. Combat : 'c', attente immobile, puis rotation de recherche.
    ----------------------------------------------------------------------------
    pc_envoie('c');
    WAIT FOR 20 us;
    verifier(led(6 DOWNTO 5) = "01", "LED du mode combat éteinte");
    moteurs('1', '1', 0, 0, "attente de 5 s");
    WAIT FOR ATTENTE_SIM * 2 us;
    moteurs('1', '0', 40, 40, "recherche : rotation sur place");

    -- 3. Adversaire vu par le capteur gauche : attaque.
    ir_gauche_b <= '0';
    WAIT FOR 300 us;
    moteurs('1', '1', 100, 100, "attaque");

    -- 4. Capteur gauche sur la bordure claire : recul.
    voie(4, CLAIR);
    WAIT FOR 300 us;
    moteurs('0', '0', 70, 70, "recul au bord du Dohyo");
    verifier(led(4) = '0', "LED du capteur gauche allumée sur le clair");
    voie(4, NOIR);
    ir_gauche_b <= '1';

    -- 5. Batterie vide : moteurs arrêtés, LED7 clignote.
    --    Délai : filtre, plus un tour d'ADC et une période PWM (300 µs).
    voie(0, BAT_BAS);
    WAIT FOR FILTRE_SIM * 2 us + 300 us;
    moteurs('1', '1', 0, 0, "batterie vide");
    WAIT UNTIL led(7) = '0' FOR 1 ms;
    WAIT UNTIL led(7) = '1' FOR 1 ms;
    verifier(led(7) = '1', "LED7 ne clignote pas (batterie vide)");
    voie(0, BAT_OK);

    ----------------------------------------------------------------------------
    -- 6. BP1 : reset, nouveau message, retour en attente.
    --    L'IP de l'ADC fige la trame SPI en cours pendant le reset, puis la
    --    termine : le modèle relève des écarts de timing sur cette seule
    --    trame. On vérifie qu'il n'y en a aucun avant BP1, ni après la fin de
    --    la trame interrompue (50 µs après le reset).
    ----------------------------------------------------------------------------
    err_avant_bp1 := err_timing;
    verifier(err_avant_bp1 = 0, "écarts de timing sur le bus de l'ADC avant BP1");
    bp1_b <= '0'; WAIT FOR 10 us; bp1_b <= '1';
    WAIT FOR 50 us;
    err_bp1 := err_timing;
    WAIT FOR 16 ms;
    verifier(nb_recus = 28 AND recu(15 TO 28) = ATTENDU, "message non renvoyé après BP1");
    verifier(led(6 DOWNTO 5) = "00", "BP1 ne remet pas en attente");
    moteurs('1', '1', 0, 0, "après BP1");

    ----------------------------------------------------------------------------
    -- 7. Suiveur : 's', ligne noire sous le capteur centre seul.
    ----------------------------------------------------------------------------
    FOR k IN 1 TO 5 LOOP
      voie(k, CLAIR);
    END LOOP;
    voie(2, NOIR);
    pc_envoie('s');
    WAIT FOR 300 us;
    verifier(led(6 DOWNTO 5) = "10", "LED du mode suiveur éteinte");
    moteurs('1', '1', 60, 60, "suivi de ligne tout droit");

    -- Ligne sous le capteur centre gauche : virage doux à gauche.
    voie(2, CLAIR);
    voie(1, NOIR);
    WAIT FOR 300 us;
    moteurs('1', '1', 25, 60, "virage doux à gauche");

    -- Aucun nouvel écart de timing SPI après la trame coupée par BP1.
    verifier(err_timing = err_bp1, "écarts de timing sur le bus de l'ADC après BP1");

    IF nb_echecs = 0 THEN
      REPORT "TEST OK : tb_robot_sumo_top (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_robot_sumo_top (" & INTEGER'IMAGE(nb_echecs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    WAIT;
  END PROCESS scenario;

END ARCHITECTURE tb_robot_sumo_top_arch;
