--------------------------------------------------------------------------------
-- Fichier      : tb_fsm_combat.vhd
-- Entité       : tb_fsm_combat
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant de la machine à états du combat.
-- Principe     : Horloge 500 kHz, attente réduite à 20 cycles. On parcourt
--                les transitions du diagramme du cahier des charges en
--                vérifiant à chaque fois le sens et la vitesse des roues :
--                arrêt hors combat, attente immobile, recherche (rotation),
--                attaque (marche avant), évitement de la bordure (recul),
--                retour au repos si le mode est quitté, arrêt définitif si
--                la batterie est vide. "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY tb_fsm_combat IS
END ENTITY tb_fsm_combat;

ARCHITECTURE tb_fsm_combat_arch OF tb_fsm_combat IS

  CONSTANT ATTENTE_SIM : NATURAL := 20;   -- attente avant combat (40 µs)

  SIGNAL clk        : STD_LOGIC := '0';
  SIGNAL rst_b      : STD_LOGIC := '0';
  SIGNAL actif      : STD_LOGIC := '0';
  SIGNAL adversaire : STD_LOGIC := '0';
  SIGNAL bordure    : STD_LOGIC := '0';
  SIGNAL batt_vide  : STD_LOGIC := '0';
  SIGNAL avance_g, avance_d   : STD_LOGIC;
  SIGNAL vitesse_g, vitesse_d : UNSIGNED(6 DOWNTO 0);
  SIGNAL fin_simu   : BOOLEAN := FALSE;

BEGIN

  u_fsm : ENTITY WORK.fsm_combat
    GENERIC MAP (ATTENTE_CYCLES_G => ATTENTE_SIM)
    PORT MAP (clk => clk, rst_b => rst_b, actif => actif, adversaire => adversaire,
              bordure => bordure, batt_vide => batt_vide,
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

    -- Vérifie la commande des deux roues.
    PROCEDURE roues (ag, ad : STD_LOGIC; vg, vd : NATURAL; situation : STRING) IS
    BEGIN
      verifier(avance_g = ag AND avance_d = ad AND vitesse_g = vg AND vitesse_d = vd,
               situation & " : sens " & STD_LOGIC'IMAGE(avance_g) & "/" & STD_LOGIC'IMAGE(avance_d)
               & ", vitesses " & INTEGER'IMAGE(TO_INTEGER(vitesse_g)) & "/"
               & INTEGER'IMAGE(TO_INTEGER(vitesse_d)));
    END PROCEDURE roues;

  BEGIN
    WAIT FOR 5 us;
    rst_b <= '1';

    -- Mode combat non sélectionné : moteurs arrêtés.
    attendre(5);
    roues('1', '1', 0, 0, "hors combat");

    -- Mode combat : attente immobile, puis recherche (rotation sur place).
    actif <= '1';
    attendre(ATTENTE_SIM / 2);
    roues('1', '1', 0, 0, "pendant l'attente de 5 s");
    attendre(ATTENTE_SIM);
    roues('1', '0', 40, 40, "recherche");

    -- Adversaire vu : attaque à pleine vitesse.
    adversaire <= '1';
    attendre(2);
    roues('1', '1', 100, 100, "attaque");

    -- Bordure pendant l'attaque : recul (la bordure prime sur l'adversaire).
    bordure <= '1';
    attendre(2);
    roues('0', '0', 70, 70, "évitement de la bordure");

    -- Fin de la bordure : retour à la recherche (adversaire perdu).
    adversaire <= '0';
    bordure    <= '0';
    attendre(2);
    roues('1', '0', 40, 40, "retour à la recherche");

    -- Bordure pendant la recherche : recul.
    bordure <= '1';
    attendre(2);
    roues('0', '0', 70, 70, "bordure pendant la recherche");
    bordure <= '0';
    attendre(2);

    -- Le PC quitte le mode combat : arrêt.
    actif <= '0';
    attendre(2);
    roues('1', '1', 0, 0, "mode combat quitté");

    -- Batterie vide pendant le combat : arrêt définitif.
    actif <= '1';
    attendre(ATTENTE_SIM + 5);
    roues('1', '0', 40, 40, "nouvelle recherche");
    batt_vide <= '1';
    attendre(2);
    roues('1', '1', 0, 0, "batterie vide");
    batt_vide  <= '0';
    adversaire <= '1';
    attendre(10);
    roues('1', '1', 0, 0, "arrêt définitif malgré le retour de la batterie");

    -- Seul le reset fait sortir de l'arrêt.
    rst_b <= '0';
    WAIT FOR 5 us;
    rst_b <= '1';
    attendre(ATTENTE_SIM + 5);
    roues('1', '1', 100, 100, "après reset : nouveau combat (adversaire vu)");

    IF nb_echecs = 0 THEN
      REPORT "TEST OK : tb_fsm_combat (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_fsm_combat (" & INTEGER'IMAGE(nb_echecs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    WAIT;
  END PROCESS verification;

END ARCHITECTURE tb_fsm_combat_arch;
