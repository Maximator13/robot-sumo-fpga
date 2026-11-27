--------------------------------------------------------------------------------
-- Fichier      : tb_pwm_gen.vhd
-- Entité       : tb_pwm_gen
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant du générateur PWM.
-- Principe     : Horloge réelle de 500 kHz. Pour plusieurs consignes
--                (0, 25, 50, 100 %), on compte sur une période complète le
--                nombre de cycles où la sortie vaut '1' : il doit être égal
--                à la consigne. On vérifie aussi la période de 200 µs (5 kHz)
--                et le reset (sortie à '0'). "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY tb_pwm_gen IS
END ENTITY tb_pwm_gen;

ARCHITECTURE tb_pwm_gen_arch OF tb_pwm_gen IS

  SIGNAL clk      : STD_LOGIC := '0';
  SIGNAL rst_b    : STD_LOGIC := '0';
  SIGNAL duty     : UNSIGNED(6 DOWNTO 0) := (OTHERS => '0');
  SIGNAL pwm      : STD_LOGIC;
  SIGNAL fin_simu : BOOLEAN := FALSE;

BEGIN

  u_pwm : ENTITY WORK.pwm_gen
    PORT MAP (clk => clk, rst_b => rst_b, duty => duty, pwm => pwm);

  -- Horloge système 500 kHz (période 2 µs).
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
    VARIABLE hauts        : NATURAL;
    VARIABLE t1, t2       : TIME;

    PROCEDURE verifier (condition : BOOLEAN; message : STRING) IS
    BEGIN
      nb_controles := nb_controles + 1;
      IF NOT condition THEN
        nb_echecs := nb_echecs + 1;
        REPORT "ECHEC : " & message SEVERITY ERROR;
      END IF;
    END PROCEDURE verifier;

    -- Applique une consigne, attend une période, puis compte les cycles à
    -- '1' sur la période suivante (100 cycles).
    PROCEDURE tester_consigne (valeur : NATURAL) IS
    BEGIN
      duty <= TO_UNSIGNED(valeur, 7);
      FOR i IN 1 TO 100 LOOP
        WAIT UNTIL RISING_EDGE(clk);
      END LOOP;
      hauts := 0;
      FOR i IN 1 TO 100 LOOP
        WAIT UNTIL RISING_EDGE(clk);
        IF pwm = '1' THEN
          hauts := hauts + 1;
        END IF;
      END LOOP;
      verifier(hauts = valeur, "consigne " & INTEGER'IMAGE(valeur) & " % : "
               & INTEGER'IMAGE(hauts) & " cycles à '1' sur 100");
    END PROCEDURE tester_consigne;

  BEGIN
    -- Reset : la sortie reste à '0'.
    WAIT FOR 10 us;
    verifier(pwm = '0', "sortie non nulle pendant le reset");
    rst_b <= '1';

    -- Rapport cyclique égal à la consigne.
    tester_consigne(0);
    tester_consigne(25);
    tester_consigne(50);
    tester_consigne(100);

    -- Période de 200 µs (5 kHz) à 50 %.
    tester_consigne(50);
    WAIT UNTIL RISING_EDGE(pwm);
    t1 := NOW;
    WAIT UNTIL RISING_EDGE(pwm);
    t2 := NOW;
    verifier(t2 - t1 = 200 us, "période PWM de " & TIME'IMAGE(t2 - t1) & " au lieu de 200 us");

    -- Reset asynchrone en fonctionnement : sortie à '0'.
    duty  <= TO_UNSIGNED(100, 7);
    WAIT FOR 400 us;
    rst_b <= '0';
    WAIT FOR 1 us;
    verifier(pwm = '0', "le reset ne coupe pas la sortie");

    IF nb_echecs = 0 THEN
      REPORT "TEST OK : tb_pwm_gen (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_pwm_gen (" & INTEGER'IMAGE(nb_echecs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    WAIT;
  END PROCESS verification;

END ARCHITECTURE tb_pwm_gen_arch;
