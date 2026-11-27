--------------------------------------------------------------------------------
-- Fichier      : tb_diviseur_frequence.vhd
-- Entité       : tb_diviseur_frequence
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant du diviseur de fréquence.
-- Principe     : Horloge 50 MHz. On vérifie que la sortie reste à '0'
--                pendant le reset, puis que sa période vaut 2 µs (500 kHz)
--                avec 1 µs à '1' et 1 µs à '0' (rapport cyclique 50 %).
--                "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY tb_diviseur_frequence IS
END ENTITY tb_diviseur_frequence;

ARCHITECTURE tb_diviseur_frequence_arch OF tb_diviseur_frequence IS

  SIGNAL clk_50   : STD_LOGIC := '0';
  SIGNAL rst_b    : STD_LOGIC := '0';
  SIGNAL clk_500k : STD_LOGIC;
  SIGNAL fin_simu : BOOLEAN   := FALSE;

BEGIN

  u_diviseur : ENTITY WORK.diviseur_frequence
    PORT MAP (clk_in => clk_50, rst_in_b => rst_b, clk_500KHz => clk_500k);

  -- Horloge 50 MHz (période 20 ns).
  horloge : PROCESS
  BEGIN
    WHILE NOT fin_simu LOOP
      clk_50 <= '0'; WAIT FOR 10 ns;
      clk_50 <= '1'; WAIT FOR 10 ns;
    END LOOP;
    WAIT;
  END PROCESS horloge;

  verification : PROCESS
    VARIABLE nb_controles : NATURAL := 0;
    VARIABLE nb_echecs    : NATURAL := 0;
    VARIABLE t1, t2, t3   : TIME;

    PROCEDURE verifier (condition : BOOLEAN; message : STRING) IS
    BEGIN
      nb_controles := nb_controles + 1;
      IF NOT condition THEN
        nb_echecs := nb_echecs + 1;
        REPORT "ECHEC : " & message SEVERITY ERROR;
      END IF;
    END PROCEDURE verifier;

  BEGIN
    -- Pendant le reset, la sortie reste à '0'.
    WAIT FOR 5 us;
    verifier(clk_500k = '0', "sortie non nulle pendant le reset");
    rst_b <= '1';

    -- Mesure d'une période complète : front montant, descendant, montant.
    WAIT UNTIL RISING_EDGE(clk_500k);
    t1 := NOW;
    WAIT UNTIL FALLING_EDGE(clk_500k);
    t2 := NOW;
    WAIT UNTIL RISING_EDGE(clk_500k);
    t3 := NOW;
    verifier(t3 - t1 = 2 us, "période de " & TIME'IMAGE(t3 - t1) & " au lieu de 2 us");
    verifier(t2 - t1 = 1 us, "état haut de " & TIME'IMAGE(t2 - t1) & " au lieu de 1 us");

    -- Le reset fige de nouveau la sortie à '0'.
    WAIT UNTIL clk_500k = '1';
    rst_b <= '0';
    WAIT FOR 1 ns;
    verifier(clk_500k = '0', "le reset ne remet pas la sortie à '0'");

    IF nb_echecs = 0 THEN
      REPORT "TEST OK : tb_diviseur_frequence (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_diviseur_frequence (" & INTEGER'IMAGE(nb_echecs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    WAIT;
  END PROCESS verification;

END ARCHITECTURE tb_diviseur_frequence_arch;
