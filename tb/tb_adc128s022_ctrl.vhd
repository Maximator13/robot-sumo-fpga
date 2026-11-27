--------------------------------------------------------------------------------
-- Fichier      : tb_adc128s022_ctrl.vhd
-- Entité       : tb_adc128s022_ctrl
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant du contrôleur SPI de l'ADC
--                ADC128S022, relié au modèle de simulation de l'ADC
--                (tb/adc128s022_model.vhd, qui vérifie les contraintes de
--                timing de la datasheet).
-- Principe     : Horloge 50 MHz. Deux ensembles contrôleur + modèle :
--                - u_adc : générique par défaut, donc réglage réel de la
--                  carte (DIV_SCLK = 16, SCLK 3,125 MHz) ;
--                - u_adc_lent : DIV_SCLK = 58 (SCLK 0,86 MHz, 50,7 k
--                  échantillons/s, limite basse de la datasheet), pour
--                  vérifier le générique.
--                Chaque voie k du modèle vaut 100 x k + 7 (valeurs toutes
--                différentes) : une mesure rangée sur la mauvaise voie (piège
--                du décalage d'une trame de l'adresse) est donc visible.
--                Un observateur vérifie à CHAQUE cycle que chaque voie
--                contient sa propre valeur (ou l'ancienne valeur juste après
--                un changement, ou 0 si elle n'a pas encore été mesurée
--                depuis le reset), et qu'une voie ne change jamais sans
--                impulsion mesure_ok (la trame ignorée après un reset ne
--                doit rien écrire). Contrôles : état de repos pendant
--                le reset, fréquence de SCLK, 8 mesures par tour, impulsion
--                d'un cycle, valeurs extrêmes (0, 4095, 0xAAA, 0x555),
--                changement de valeur, resets à 5 instants différents d'une
--                trame (l'ADC garde alors une adresse quelconque), sorties
--                registrées (noms externes), aucune violation de timing.
--                Chien de garde à 5 ms. "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY tb_adc128s022_ctrl IS
END ENTITY tb_adc128s022_ctrl;

ARCHITECTURE tb_adc128s022_ctrl_arch OF tb_adc128s022_ctrl IS

  CONSTANT PERIODE_CLK : TIME := 20 ns;                  -- 50 MHz
  CONSTANT DIV_REEL    : NATURAL := 16;
  CONSTANT DIV_LENT    : NATURAL := 58;   -- limite basse : 0,86 MHz, 50,7 k échantillons/s
  CONSTANT CYCLES_TOUR : NATURAL := 8 * 17 * DIV_REEL;   -- 8 trames de 17 périodes SCLK
  CONSTANT CYCLES_TOUR_LENT : NATURAL := 8 * 17 * DIV_LENT;

  FUNCTION valeurs_initiales RETURN t_voies_adc IS
    VARIABLE v : t_voies_adc;
  BEGIN
    FOR k IN 0 TO 7 LOOP
      v(k) := TO_UNSIGNED(100 * k + 7, 12);
    END LOOP;
    RETURN v;
  END FUNCTION valeurs_initiales;

  SIGNAL clk      : STD_LOGIC := '0';
  SIGNAL rst_b    : STD_LOGIC := '0';
  SIGNAL fin_simu : BOOLEAN   := FALSE;

  -- Ensemble réel.
  SIGNAL cs_n, saddr, sclk, sdat : STD_LOGIC;
  SIGNAL voies      : t_voies_adc;
  SIGNAL mesure_ok  : STD_LOGIC;
  SIGNAL valeurs    : t_voies_adc := valeurs_initiales;
  SIGNAL anciennes  : t_voies_adc := valeurs_initiales;   -- valeurs avant le dernier changement
  SIGNAL nb_conv, err_timing, incompletes : NATURAL;

  -- Ensemble lent.
  SIGNAL cs_n_l, saddr_l, sclk_l, sdat_l : STD_LOGIC;
  SIGNAL voies_l     : t_voies_adc;
  SIGNAL mesure_ok_l : STD_LOGIC;
  SIGNAL nb_conv_l, err_timing_l, incompletes_l : NATURAL;

  -- Observateurs.
  SIGNAL nb_mesures      : NATURAL := 0;
  SIGNAL erreurs_obs     : NATURAL := 0;
  SIGNAL erreurs_struct  : NATURAL := 0;

BEGIN

  -- Générique laissé à sa valeur par défaut : c'est ce réglage qui ira sur
  -- la carte, il doit donner SCLK = 3,125 MHz (DIV_REEL = 16).
  u_adc128s022_ctrl : ENTITY WORK.adc128s022_ctrl
    PORT MAP (clk => clk, rst_b => rst_b, adc_cs_n => cs_n, adc_saddr => saddr,
              adc_sclk => sclk, adc_sdat => sdat, voies => voies, mesure_ok => mesure_ok);

  u_modele : ENTITY WORK.adc128s022_model
    PORT MAP (cs_n => cs_n, sclk => sclk, din => saddr, dout => sdat, valeurs => valeurs,
              nb_conversions => nb_conv, erreurs_timing => err_timing, trames_incompletes => incompletes);

  u_adc_lent : ENTITY WORK.adc128s022_ctrl
    GENERIC MAP (DIV_SCLK => DIV_LENT)
    PORT MAP (clk => clk, rst_b => rst_b, adc_cs_n => cs_n_l, adc_saddr => saddr_l,
              adc_sclk => sclk_l, adc_sdat => sdat_l, voies => voies_l, mesure_ok => mesure_ok_l);

  u_modele_lent : ENTITY WORK.adc128s022_model
    PORT MAP (cs_n => cs_n_l, sclk => sclk_l, din => saddr_l, dout => sdat_l, valeurs => valeurs_initiales,
              nb_conversions => nb_conv_l, erreurs_timing => err_timing_l, trames_incompletes => incompletes_l);

  horloge : PROCESS
  BEGIN
    WHILE NOT fin_simu LOOP
      clk <= '0'; WAIT FOR PERIODE_CLK / 2;
      clk <= '1'; WAIT FOR PERIODE_CLK / 2;
    END LOOP;
    WAIT;
  END PROCESS horloge;

  -- À chaque nouvelle mesure, chaque voie doit contenir sa propre valeur.
  -- Une impulsion mesure_ok dure un seul cycle.
  observateur : PROCESS (clk, rst_b)
    TYPE t_drapeaux IS ARRAY (0 TO 7) OF BOOLEAN;
    VARIABLE deja_mesuree : t_drapeaux := (OTHERS => FALSE);
    VARIABLE precedent : STD_LOGIC := '0';
    VARIABLE voies_prec : t_voies_adc := (OTHERS => (OTHERS => '0'));
  BEGIN
    IF rst_b = '0' THEN
      deja_mesuree := (OTHERS => FALSE);
      precedent := '0';
      voies_prec := (OTHERS => (OTHERS => '0'));
    ELSIF RISING_EDGE(clk) THEN
      IF mesure_ok = '1' THEN
        nb_mesures <= nb_mesures + 1;
        IF precedent = '1' THEN
          erreurs_obs <= erreurs_obs + 1;
          REPORT "ECHEC : mesure_ok dure plus d'un cycle" SEVERITY ERROR;
        END IF;
      ELSIF voies /= voies_prec THEN
        erreurs_obs <= erreurs_obs + 1;
        REPORT "ECHEC : une voie a changé sans impulsion mesure_ok" SEVERITY ERROR;
      END IF;
      -- Contrôle du contenu de chaque voie, à chaque cycle.
      FOR k IN 0 TO 7 LOOP
          IF voies(k) /= 0 THEN
            deja_mesuree(k) := TRUE;
          END IF;
          IF NOT (voies(k) = valeurs(k) OR voies(k) = anciennes(k) OR (voies(k) = 0 AND NOT deja_mesuree(k))) THEN
            erreurs_obs <= erreurs_obs + 1;
            REPORT "ECHEC : la voie " & INTEGER'IMAGE(k) & " contient " & INTEGER'IMAGE(TO_INTEGER(voies(k))) &
                   " au lieu de " & INTEGER'IMAGE(TO_INTEGER(valeurs(k))) & " (mesure rangée sur la mauvaise voie ?)" SEVERITY ERROR;
          END IF;
        END LOOP;
      precedent := mesure_ok;
      voies_prec := voies;
    END IF;
  END PROCESS observateur;

  -- Contrôle de structure : les sorties SPI sont les bascules internes.
  sorties_registrees : PROCESS
    ALIAS cs_reg    IS << SIGNAL .tb_adc128s022_ctrl.u_adc128s022_ctrl.cs_n_reg  : STD_LOGIC >>;
    ALIAS sclk_reg  IS << SIGNAL .tb_adc128s022_ctrl.u_adc128s022_ctrl.sclk_reg  : STD_LOGIC >>;
    ALIAS saddr_reg IS << SIGNAL .tb_adc128s022_ctrl.u_adc128s022_ctrl.saddr_reg : STD_LOGIC >>;
  BEGIN
    WAIT ON clk;
    WAIT FOR 1 ns;
    IF cs_n /= cs_reg OR sclk /= sclk_reg OR saddr /= saddr_reg THEN
      erreurs_struct <= erreurs_struct + 1;
      REPORT "ECHEC : une sortie SPI n'est pas la sortie directe de sa bascule" SEVERITY ERROR;
    END IF;
  END PROCESS sorties_registrees;

  chien_de_garde : PROCESS
  BEGIN
    WAIT FOR 5 ms;
    IF NOT fin_simu THEN
      REPORT "TEST KO : tb_adc128s022_ctrl bloqué (délai de 5 ms dépassé)" SEVERITY FAILURE;
    END IF;
    WAIT;
  END PROCESS chien_de_garde;

  verification : PROCESS
    VARIABLE nb_controles : NATURAL := 0;
    VARIABLE nb_echecs    : NATURAL := 0;
    VARIABLE t0           : TIME;
    VARIABLE n0, inc0     : NATURAL;
    VARIABLE nb_montants  : NATURAL;

    PROCEDURE verifier (condition : BOOLEAN; message : STRING) IS
    BEGIN
      nb_controles := nb_controles + 1;
      IF NOT condition THEN
        nb_echecs := nb_echecs + 1;
      END IF;
      ASSERT condition REPORT "ECHEC : " & message SEVERITY ERROR;
    END PROCEDURE verifier;

    PROCEDURE attendre_cycles (n : NATURAL) IS
    BEGIN
      FOR i IN 1 TO n LOOP
        WAIT UNTIL RISING_EDGE(clk);
      END LOOP;
      WAIT FOR 1 ns;
    END PROCEDURE attendre_cycles;

    -- Toutes les voies du contrôleur réel égales aux valeurs du modèle.
    PROCEDURE verifier_toutes_voies (contexte : STRING) IS
    BEGIN
      FOR k IN 0 TO 7 LOOP
        verifier(voies(k) = valeurs(k), contexte & " : voie " & INTEGER'IMAGE(k) & " = " &
                 INTEGER'IMAGE(TO_INTEGER(voies(k))) & " au lieu de " & INTEGER'IMAGE(TO_INTEGER(valeurs(k))));
      END LOOP;
    END PROCEDURE verifier_toutes_voies;

  BEGIN
    ----------------------------------------------------------------------------
    -- 1. Pendant le reset : ADC désélectionné, SCLK au repos, voies à 0.
    ----------------------------------------------------------------------------
    FOR i IN 1 TO 50 LOOP
      attendre_cycles(1);
      verifier(cs_n = '1' AND sclk = '1' AND mesure_ok = '0', "bus SPI actif pendant le reset");
    END LOOP;
    verifier(voies(3) = 0, "voies non nulles pendant le reset");
    rst_b <= '1';

    ----------------------------------------------------------------------------
    -- 2. Fréquence de SCLK et forme de la première trame.
    ----------------------------------------------------------------------------
    WAIT UNTIL FALLING_EDGE(cs_n);
    verifier(sclk = '1', "SCLK pas au repos quand CS descend");
    nb_montants := 0;
    WAIT UNTIL RISING_EDGE(sclk);
    t0 := NOW;
    nb_montants := 1;
    WAIT UNTIL RISING_EDGE(sclk);
    verifier(NOW - t0 = DIV_REEL * PERIODE_CLK,
             "période SCLK de " & TIME'IMAGE(NOW - t0) & " au lieu de 320 ns (3,125 MHz)");
    verifier(NOW - t0 >= 312.5 ns AND NOW - t0 <= 1250 ns, "SCLK hors de la plage 0,8 - 3,2 MHz de l'ADC");
    nb_montants := 2;
    WHILE cs_n = '0' LOOP
      WAIT UNTIL RISING_EDGE(sclk) OR RISING_EDGE(cs_n);
      IF RISING_EDGE(sclk) THEN
        nb_montants := nb_montants + 1;
      END IF;
    END LOOP;
    verifier(nb_montants = 16, INTEGER'IMAGE(nb_montants) & " fronts montants de SCLK dans la trame au lieu de 16");
    verifier(sclk = '1', "SCLK pas au repos quand CS remonte");

    ----------------------------------------------------------------------------
    -- 3. Après 3 tours, chaque voie contient sa valeur ; puis exactement 8
    --    mesures par tour.
    ----------------------------------------------------------------------------
    attendre_cycles(3 * CYCLES_TOUR);
    verifier_toutes_voies("après 3 tours");
    n0 := nb_mesures;
    attendre_cycles(CYCLES_TOUR);
    verifier(nb_mesures - n0 = 8, INTEGER'IMAGE(nb_mesures - n0) & " mesures en un tour au lieu de 8");

    ----------------------------------------------------------------------------
    -- 4. Valeurs extrêmes et motifs alternés (tous les bits testés).
    ----------------------------------------------------------------------------
    anciennes <= valeurs;
    valeurs(0) <= TO_UNSIGNED(1, 12);
    valeurs(3) <= TO_UNSIGNED(4095, 12);
    valeurs(4) <= TO_UNSIGNED(0, 12);
    valeurs(5) <= x"AAA";
    valeurs(6) <= x"555";
    valeurs(7) <= x"800";
    attendre_cycles(3 * CYCLES_TOUR);
    verifier_toutes_voies("valeurs extrêmes");
    anciennes <= valeurs;

    ----------------------------------------------------------------------------
    -- 5. Resets à 5 instants différents d'une trame : l'ADC garde une adresse
    --    quelconque ; aucune mesure ne doit être rangée sur la mauvaise voie
    --    (observateur) et tout redevient juste ensuite.
    ----------------------------------------------------------------------------
    inc0 := incompletes;
    FOR essai IN 0 TO 4 LOOP
      WAIT UNTIL FALLING_EDGE(cs_n);
      attendre_cycles(17 + essai * 50);       -- au milieu de la trame, à des rangs variés
      rst_b <= '0';
      attendre_cycles(10);
      verifier(cs_n = '1' AND sclk = '1', "le reset ne libère pas le bus SPI");
      verifier(voies(5) = 0, "le reset ne remet pas les voies à 0");
      rst_b <= '1';
      attendre_cycles(3 * CYCLES_TOUR);
      verifier_toutes_voies("après reset n" & INTEGER'IMAGE(essai));
    END LOOP;
    verifier(incompletes - inc0 = 5, "les resets devaient interrompre 5 trames");

    ----------------------------------------------------------------------------
    -- 6. Exemplaire lent (SCLK à 0,86 MHz) : toutes les voies justes.
    ----------------------------------------------------------------------------
    attendre_cycles(3 * CYCLES_TOUR_LENT);
    FOR k IN 0 TO 7 LOOP
      verifier(voies_l(k) = TO_UNSIGNED(100 * k + 7, 12),
               "SCLK lente : voie " & INTEGER'IMAGE(k) & " = " & INTEGER'IMAGE(TO_INTEGER(voies_l(k))));
    END LOOP;

    ----------------------------------------------------------------------------
    -- 7. Aucune violation de timing vue par les modèles d'ADC.
    ----------------------------------------------------------------------------
    verifier(err_timing = 0, INTEGER'IMAGE(err_timing) & " violation(s) de timing (contrôleur réel)");
    verifier(err_timing_l = 0, INTEGER'IMAGE(err_timing_l) & " violation(s) de timing (contrôleur lent)");
    verifier(nb_conv > 100, "trop peu de conversions effectuées");

    -- Bilan.
    IF nb_echecs = 0 AND erreurs_obs = 0 AND erreurs_struct = 0 THEN
      REPORT "TEST OK : tb_adc128s022_ctrl (" & INTEGER'IMAGE(nb_controles) & " controles, " &
             INTEGER'IMAGE(nb_mesures) & " mesures observées)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_adc128s022_ctrl (" & INTEGER'IMAGE(nb_echecs + erreurs_obs + erreurs_struct) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    std.env.finish;
  END PROCESS verification;

END ARCHITECTURE tb_adc128s022_ctrl_arch;
