--------------------------------------------------------------------------------
-- Fichier      : tb_suiveur_ligne.vhd
-- Entité       : tb_suiveur_ligne
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant du suiveur de ligne.
-- Principe     : Horloge 500 kHz, détection d'extrémité réduite à 10 cycles
--                de blanc. On impose la position de la ligne sous les 5
--                capteurs et on vérifie la commande des roues : tout droit,
--                virages doux, pivots, perte de ligne dans un virage serré,
--                extrémité de ligne puis demi-tour, arrêt après 4 extrémités
--                (2 allers / retours), arrêt si la batterie est vide.
--                "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY tb_suiveur_ligne IS
END ENTITY tb_suiveur_ligne;

ARCHITECTURE tb_suiveur_ligne_arch OF tb_suiveur_ligne IS

  CONSTANT FIN_SIM : NATURAL := 10;   -- cycles de blanc = extrémité (20 µs)

  SIGNAL clk       : STD_LOGIC := '0';
  SIGNAL rst_b     : STD_LOGIC := '0';
  SIGNAL actif     : STD_LOGIC := '0';
  SIGNAL sol_noir  : STD_LOGIC_VECTOR(4 DOWNTO 0) := "00100";
  SIGNAL batt_vide : STD_LOGIC := '0';
  SIGNAL avance_g, avance_d   : STD_LOGIC;
  SIGNAL vitesse_g, vitesse_d : UNSIGNED(6 DOWNTO 0);
  SIGNAL fin_simu  : BOOLEAN := FALSE;

BEGIN

  u_suiveur : ENTITY WORK.suiveur_ligne
    GENERIC MAP (FIN_LIGNE_CYCLES_G => FIN_SIM)
    PORT MAP (clk => clk, rst_b => rst_b, actif => actif, sol_noir => sol_noir,
              batt_vide => batt_vide,
              avance_gauche => avance_g, avance_droite => avance_d,
              vitesse_gauche => vitesse_g, vitesse_droite => vitesse_d);

  horloge : PROCESS
  BEGIN
    WHILE NOT fin_simu LOOP
      clk <= '0'; WAIT FOR 1 us;
      clk <= '1'; WAIT FOR 1 us;
    END LOOP;
    WAIT;
  END PROCESS horloge;

  verification : PROCESS
    VARIABLE nb_controles : NATURAL := 0;
    VARIABLE nb_echecs    : NATURAL := 0;

    PROCEDURE verifier (condition : BOOLEAN; message : STRING) IS
    BEGIN
      nb_controles := nb_controles + 1;
      IF NOT condition THEN
        nb_echecs := nb_echecs + 1;
        REPORT "ECHEC : " & message SEVERITY ERROR;
      END IF;
    END PROCEDURE verifier;

    PROCEDURE attendre (n : NATURAL) IS
    BEGIN
      FOR i IN 1 TO n LOOP
        WAIT UNTIL RISING_EDGE(clk);
      END LOOP;
      WAIT FOR 100 ns;
    END PROCEDURE attendre;

    -- Impose la position de la ligne et vérifie la commande des roues.
    PROCEDURE tester (capteurs : STD_LOGIC_VECTOR(4 DOWNTO 0); ag, ad : STD_LOGIC;
                      vg, vd : NATURAL; situation : STRING) IS
    BEGIN
      sol_noir <= capteurs;
      attendre(3);
      verifier(avance_g = ag AND avance_d = ad AND vitesse_g = vg AND vitesse_d = vd,
               situation & " : sens " & STD_LOGIC'IMAGE(avance_g) & "/" & STD_LOGIC'IMAGE(avance_d)
               & ", vitesses " & INTEGER'IMAGE(TO_INTEGER(vitesse_g)) & "/"
               & INTEGER'IMAGE(TO_INTEGER(vitesse_d)));
    END PROCEDURE tester;

    -- Simule une extrémité de ligne puis le demi-tour jusqu'au centre.
    PROCEDURE extremite IS
    BEGIN
      sol_noir <= "00100";
      attendre(3);
      sol_noir <= "00000";
      attendre(FIN_SIM + 3);
      sol_noir <= "00100";
      attendre(3);
    END PROCEDURE extremite;

  BEGIN
    WAIT FOR 5 us;
    rst_b <= '1';

    -- Mode non sélectionné : moteurs arrêtés.
    attendre(5);
    verifier(vitesse_g = 0 AND vitesse_d = 0, "moteurs actifs hors mode suiveur");

    -- Corrections selon la position de la ligne.
    actif <= '1';
    tester("00100", '1', '1', 60, 60, "ligne au centre : tout droit");
    tester("01000", '1', '1', 25, 60, "ligne au centre gauche : virage doux à gauche");
    tester("10000", '0', '1', 60, 60, "ligne à gauche : pivot à gauche");
    tester("00010", '1', '1', 60, 25, "ligne au centre droit : virage doux à droite");
    tester("00001", '1', '0', 60, 60, "ligne à droite : pivot à droite");
    tester("11111", '1', '1', 60, 60, "croisement tout noir : tout droit");

    -- Perte de ligne dans un virage serré à gauche : pivot vers la gauche.
    tester("01000", '1', '1', 25, 60, "virage doux avant la perte");
    tester("00000", '0', '1', 60, 60, "perte de ligne à gauche : pivot à gauche");
    attendre(FIN_SIM * 2);
    verifier(avance_g = '0' AND avance_d = '1', "virage serré pris pour une extrémité");
    tester("00100", '1', '1', 60, 60, "ligne retrouvée");

    -- Perte de ligne au centre : avance lente, puis demi-tour (pivot droit).
    tester("00000", '1', '1', 25, 25, "perte au centre : avance lente");
    attendre(FIN_SIM);
    verifier(avance_g = '1' AND avance_d = '0' AND vitesse_g = 60, "extrémité : pas de demi-tour");
    tester("00100", '1', '1', 60, 60, "fin du demi-tour : suivi");

    -- Trois autres extrémités : arrêt à la 4e (2 allers / retours).
    extremite;
    extremite;
    verifier(vitesse_g = 60 AND vitesse_d = 60, "arrêt avant la 4e extrémité");
    sol_noir <= "00000";
    attendre(FIN_SIM + 3);
    verifier(vitesse_g = 0 AND vitesse_d = 0, "pas d'arrêt à la 4e extrémité");
    sol_noir <= "00100";
    attendre(5);
    verifier(vitesse_g = 0 AND vitesse_d = 0, "le robot repart après la fin du parcours");

    -- Nouvelle activation : parcours complet, puis batterie vide.
    actif <= '0';
    attendre(2);
    actif <= '1';
    tester("00100", '1', '1', 60, 60, "nouveau parcours");
    batt_vide <= '1';
    attendre(2);
    verifier(vitesse_g = 0 AND vitesse_d = 0, "batterie vide : moteurs non arrêtés");

    IF nb_echecs = 0 THEN
      REPORT "TEST OK : tb_suiveur_ligne (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_suiveur_ligne (" & INTEGER'IMAGE(nb_echecs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    WAIT;
  END PROCESS verification;

END ARCHITECTURE tb_suiveur_ligne_arch;
