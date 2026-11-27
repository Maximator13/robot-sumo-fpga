--------------------------------------------------------------------------------
-- Fichier      : tb_detection_capteurs.vhd
-- Entité       : tb_detection_capteurs
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant de la mise en forme des capteurs.
-- Principe     : Horloge 500 kHz, maintien de la bordure réduit à 20 cycles.
--                On vérifie : le seuil noir / clair de chaque capteur de sol
--                (bon bit de sol_noir), le signal bordure et son maintien,
--                la détection de l'adversaire par chacun des deux capteurs
--                infrarouges (actifs à '0'), et la recopie du code batterie.
--                "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY tb_detection_capteurs IS
END ENTITY tb_detection_capteurs;

ARCHITECTURE tb_detection_capteurs_arch OF tb_detection_capteurs IS

  CONSTANT MAINTIEN_SIM : NATURAL := 20;   -- maintien de la bordure (40 µs)
  -- Codes ADC : noir au-dessus du seuil de 2048, clair en dessous.
  CONSTANT NOIR  : STD_LOGIC_VECTOR(11 DOWNTO 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(3200, 12));
  CONSTANT CLAIR : STD_LOGIC_VECTOR(11 DOWNTO 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(400, 12));

  SIGNAL clk           : STD_LOGIC := '0';
  SIGNAL rst_b         : STD_LOGIC := '0';
  SIGNAL ch_bat        : STD_LOGIC_VECTOR(11 DOWNTO 0) := STD_LOGIC_VECTOR(TO_UNSIGNED(3000, 12));
  SIGNAL ch_g, ch_cg, ch_c, ch_cd, ch_d : STD_LOGIC_VECTOR(11 DOWNTO 0) := NOIR;
  SIGNAL ir_g_b, ir_d_b : STD_LOGIC := '1';
  SIGNAL sol_noir      : STD_LOGIC_VECTOR(4 DOWNTO 0);
  SIGNAL bordure       : STD_LOGIC;
  SIGNAL adversaire    : STD_LOGIC;
  SIGNAL code_batterie : UNSIGNED(11 DOWNTO 0);
  SIGNAL fin_simu      : BOOLEAN := FALSE;

BEGIN

  u_detection : ENTITY WORK.detection_capteurs
    GENERIC MAP (BORDURE_MAINTIEN_CYCLES_G => MAINTIEN_SIM)
    PORT MAP (clk => clk, rst_b => rst_b,
              ch_batterie => ch_bat, ch_sol_centre_gauche => ch_cg,
              ch_sol_centre => ch_c, ch_sol_centre_droit => ch_cd,
              ch_sol_gauche => ch_g, ch_sol_droit => ch_d,
              ir_gauche_b => ir_g_b, ir_droit_b => ir_d_b,
              sol_noir => sol_noir, bordure => bordure,
              adversaire => adversaire, code_batterie => code_batterie);

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

    -- Laisse passer quelques cycles (recopie + calcul).
    PROCEDURE attendre (n : NATURAL) IS
    BEGIN
      FOR i IN 1 TO n LOOP
        WAIT UNTIL RISING_EDGE(clk);
      END LOOP;
      WAIT FOR 100 ns;
    END PROCEDURE attendre;

  BEGIN
    WAIT FOR 5 us;
    verifier(bordure = '1', "bordure inactive pendant le reset (prudence)");
    rst_b <= '1';

    -- Sol tout noir : 5 bits à '1', bordure retombée après le maintien.
    attendre(MAINTIEN_SIM + 5);
    verifier(sol_noir = "11111", "sol noir non détecté sous les 5 capteurs");
    verifier(bordure = '0', "bordure active alors que tout est noir");
    verifier(code_batterie = 3000, "code batterie mal recopié");

    -- Chaque capteur passe au clair à son tour : bon bit, bordure active.
    ch_g <= CLAIR;  attendre(3);
    verifier(sol_noir = "01111" AND bordure = '1', "capteur gauche (bit 4) ou bordure incorrect");
    ch_g <= NOIR;   ch_cg <= CLAIR; attendre(3);
    verifier(sol_noir = "10111", "capteur centre gauche (bit 3) incorrect");
    ch_cg <= NOIR;  ch_c <= CLAIR;  attendre(3);
    verifier(sol_noir = "11011", "capteur centre (bit 2) incorrect");
    ch_c <= NOIR;   ch_cd <= CLAIR; attendre(3);
    verifier(sol_noir = "11101", "capteur centre droit (bit 1) incorrect");
    ch_cd <= NOIR;  ch_d <= CLAIR;  attendre(3);
    verifier(sol_noir = "11110", "capteur droit (bit 0) incorrect");

    -- Retour au noir : la bordure reste active pendant le maintien.
    ch_d <= NOIR;
    attendre(MAINTIEN_SIM / 2);
    verifier(bordure = '1', "bordure retombée avant la fin du maintien");
    attendre(MAINTIEN_SIM);
    verifier(bordure = '0', "bordure toujours active après le maintien");

    -- Capteurs infrarouges : '0' = obstacle, détecté après synchronisation.
    verifier(adversaire = '0', "adversaire détecté sans obstacle");
    ir_g_b <= '0'; attendre(3);
    verifier(adversaire = '1', "adversaire non détecté par le capteur gauche");
    ir_g_b <= '1'; attendre(3);
    verifier(adversaire = '0', "adversaire encore détecté après disparition");
    ir_d_b <= '0'; attendre(3);
    verifier(adversaire = '1', "adversaire non détecté par le capteur droit");

    IF nb_echecs = 0 THEN
      REPORT "TEST OK : tb_detection_capteurs (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_detection_capteurs (" & INTEGER'IMAGE(nb_echecs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    WAIT;
  END PROCESS verification;

END ARCHITECTURE tb_detection_capteurs_arch;
