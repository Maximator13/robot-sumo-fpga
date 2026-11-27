--------------------------------------------------------------------------------
-- Fichier      : tb_batterie_monitor.vhd
-- Entité       : tb_batterie_monitor
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant de la surveillance batterie.
-- Principe     : Horloge 500 kHz, filtre réduit à 50 cycles et demi-période
--                de clignotement à 30 cycles. On mesure le rapport cyclique
--                de LED7 (cycles à '1' sur une période de 100) :
--                9 V -> 100 %, 7,2 V -> 10 %, 8,1 V -> environ 55 %.
--                On vérifie ensuite qu'une baisse brève ne déclare pas la
--                batterie vide, qu'une baisse prolongée le fait (LED7 clignote
--                avec la bonne période) et que l'état reste définitif.
--                "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY tb_batterie_monitor IS
END ENTITY tb_batterie_monitor;

ARCHITECTURE tb_batterie_monitor_arch OF tb_batterie_monitor IS

  CONSTANT FILTRE_SIM : NATURAL := 50;   -- 100 µs sous le seuil avant batterie vide
  CONSTANT CLIGNO_SIM : NATURAL := 30;   -- demi-période de clignotement (60 µs)

  SIGNAL clk       : STD_LOGIC := '0';
  SIGNAL rst_b     : STD_LOGIC := '0';
  SIGNAL code      : UNSIGNED(11 DOWNTO 0) := TO_UNSIGNED(3434, 12);
  SIGNAL led7      : STD_LOGIC;
  SIGNAL batt_vide : STD_LOGIC;
  SIGNAL fin_simu  : BOOLEAN := FALSE;

BEGIN

  u_batterie : ENTITY WORK.batterie_monitor
    GENERIC MAP (CLIGNO_DEMI_CYCLES_G => CLIGNO_SIM, FILTRE_BATT_CYCLES_G => FILTRE_SIM)
    PORT MAP (clk => clk, rst_b => rst_b, code_batterie => code,
              led7 => led7, batt_vide => batt_vide);

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

    -- Impose un code, attend 2 périodes PWM, puis compte les cycles où LED7
    -- est allumée sur une période (100 cycles).
    PROCEDURE mesurer_led (valeur : NATURAL) IS
    BEGIN
      code <= TO_UNSIGNED(valeur, 12);
      FOR i IN 1 TO 200 LOOP
        WAIT UNTIL RISING_EDGE(clk);
      END LOOP;
      hauts := 0;
      FOR i IN 1 TO 100 LOOP
        WAIT UNTIL RISING_EDGE(clk);
        IF led7 = '1' THEN
          hauts := hauts + 1;
        END IF;
      END LOOP;
    END PROCEDURE mesurer_led;

  BEGIN
    WAIT FOR 5 us;
    rst_b <= '1';

    -- Luminosité proportionnelle à la tension.
    mesurer_led(3434);   -- 9 V
    verifier(hauts = 100, "9 V : LED7 à " & INTEGER'IMAGE(hauts) & " % au lieu de 100 %");
    mesurer_led(4000);   -- au-dessus de 9 V
    verifier(hauts = 100, "au-dessus de 9 V : LED7 à " & INTEGER'IMAGE(hauts) & " %");
    mesurer_led(2689);   -- 7,2 V
    verifier(hauts = 10, "7,2 V : LED7 à " & INTEGER'IMAGE(hauts) & " % au lieu de 10 %");
    mesurer_led(3062);   -- environ 8,1 V
    verifier(hauts >= 54 AND hauts <= 56, "8,1 V : LED7 à " & INTEGER'IMAGE(hauts) & " % au lieu de 55 %");

    -- Baisse brève (moins que le filtre) : batterie pas déclarée vide.
    code <= TO_UNSIGNED(2400, 12);
    FOR i IN 1 TO FILTRE_SIM / 2 LOOP
      WAIT UNTIL RISING_EDGE(clk);
    END LOOP;
    code <= TO_UNSIGNED(3000, 12);
    FOR i IN 1 TO FILTRE_SIM LOOP
      WAIT UNTIL RISING_EDGE(clk);
    END LOOP;
    verifier(batt_vide = '0', "batterie déclarée vide sur une baisse brève");

    -- Baisse prolongée : batterie vide, LED7 clignote.
    code <= TO_UNSIGNED(2400, 12);
    FOR i IN 1 TO FILTRE_SIM + 5 LOOP
      WAIT UNTIL RISING_EDGE(clk);
    END LOOP;
    verifier(batt_vide = '1', "batterie non déclarée vide après le filtre");
    WAIT UNTIL RISING_EDGE(led7);
    t1 := NOW;
    WAIT UNTIL RISING_EDGE(led7);
    t2 := NOW;
    verifier(t2 - t1 = 2 * CLIGNO_SIM * 2 us, "période de clignotement de " & TIME'IMAGE(t2 - t1));

    -- La tension remonte : la batterie reste déclarée vide jusqu'au reset.
    code <= TO_UNSIGNED(3434, 12);
    WAIT FOR 200 us;
    verifier(batt_vide = '1', "batterie vide non définitive");
    rst_b <= '0';
    WAIT FOR 5 us;
    verifier(batt_vide = '0', "le reset n'efface pas la batterie vide");

    IF nb_echecs = 0 THEN
      REPORT "TEST OK : tb_batterie_monitor (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_batterie_monitor (" & INTEGER'IMAGE(nb_echecs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    WAIT;
  END PROCESS verification;

END ARCHITECTURE tb_batterie_monitor_arch;
