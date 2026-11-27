--------------------------------------------------------------------------------
-- Fichier      : tb_circuit_reset.vhd
-- Entité       : tb_circuit_reset
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant du circuit de reset.
-- Principe     : Horloge 50 MHz, durée du reset réduite à 100 cycles
--                (2 µs) pour simuler vite. On vérifie :
--                - mise sous tension : rst_b à '0' puis à '1' après la durée ;
--                - appui sur BP1 : rst_b repasse à '0' et y reste tant que le
--                  bouton est enfoncé, puis remonte après la durée ;
--                - la valeur réelle par défaut (50 000 cycles = 1 ms).
--                "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY tb_circuit_reset IS
END ENTITY tb_circuit_reset;

ARCHITECTURE tb_circuit_reset_arch OF tb_circuit_reset IS

  CONSTANT DUREE_SIM : NATURAL := 100;   -- durée du reset en simulation (2 µs)

  SIGNAL clk_50     : STD_LOGIC := '0';
  SIGNAL bp1_b      : STD_LOGIC := '1';
  SIGNAL rst_b      : STD_LOGIC;
  SIGNAL rst_b_reel : STD_LOGIC;
  SIGNAL fin_simu   : BOOLEAN   := FALSE;

BEGIN

  -- Exemplaire à durée réduite, et exemplaire à la valeur réelle (1 ms).
  u_reset : ENTITY WORK.circuit_reset
    GENERIC MAP (RESET_CYCLES_G => DUREE_SIM)
    PORT MAP (clk_in => clk_50, bp_in => bp1_b, rst_b => rst_b);

  u_reset_reel : ENTITY WORK.circuit_reset
    PORT MAP (clk_in => clk_50, bp_in => '1', rst_b => rst_b_reel);

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
    VARIABLE t_relache    : TIME;

    PROCEDURE verifier (condition : BOOLEAN; message : STRING) IS
    BEGIN
      nb_controles := nb_controles + 1;
      IF NOT condition THEN
        nb_echecs := nb_echecs + 1;
        REPORT "ECHEC : " & message SEVERITY ERROR;
      END IF;
    END PROCEDURE verifier;

  BEGIN
    -- Mise sous tension : reset actif, puis libéré après environ 2 µs.
    WAIT FOR 1 us;
    verifier(rst_b = '0', "reset inactif à la mise sous tension");
    WAIT FOR 1.5 us;
    verifier(rst_b = '1', "reset toujours actif après la durée prévue");

    -- Appui sur BP1 : reset actif tant que le bouton est enfoncé.
    bp1_b <= '0';
    WAIT FOR 200 ns;
    verifier(rst_b = '0', "l'appui sur BP1 ne déclenche pas le reset");
    WAIT FOR 5 us;
    verifier(rst_b = '0', "reset relâché alors que BP1 est encore enfoncé");

    -- Relâchement : le reset remonte environ 2 µs plus tard.
    bp1_b <= '1';
    t_relache := NOW;
    WAIT UNTIL rst_b = '1' FOR 10 us;
    verifier(NOW - t_relache >= 2 us AND NOW - t_relache < 2.2 us,
             "reset libéré " & TIME'IMAGE(NOW - t_relache) & " après le relâchement");

    -- Valeur réelle : 1 ms de reset à la mise sous tension.
    WAIT FOR 990 us - NOW;
    verifier(rst_b_reel = '0', "valeur réelle : reset trop court");
    WAIT FOR 20 us;
    verifier(rst_b_reel = '1', "valeur réelle : reset trop long");

    IF nb_echecs = 0 THEN
      REPORT "TEST OK : tb_circuit_reset (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_circuit_reset (" & INTEGER'IMAGE(nb_echecs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    WAIT;
  END PROCESS verification;

END ARCHITECTURE tb_circuit_reset_arch;
