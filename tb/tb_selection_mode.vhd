--------------------------------------------------------------------------------
-- Fichier      : tb_selection_mode.vhd
-- Entité       : tb_selection_mode
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant de la sélection du mode.
-- Principe     : On simule la sortie de UART_RX (impulsion rx_dv d'un cycle
--                avec l'octet reçu) et on vérifie : aucun mode après reset,
--                'c' -> combat, 's' -> suiveur, autre caractère ignoré,
--                retour en attente par le reset. "TEST OK" si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY tb_selection_mode IS
END ENTITY tb_selection_mode;

ARCHITECTURE tb_selection_mode_arch OF tb_selection_mode IS

  SIGNAL clk          : STD_LOGIC := '0';
  SIGNAL rst_b        : STD_LOGIC := '0';
  SIGNAL rx_dv        : STD_LOGIC := '0';
  SIGNAL rx_octet     : STD_LOGIC_VECTOR(7 DOWNTO 0) := (OTHERS => '0');
  SIGNAL mode_combat  : STD_LOGIC;
  SIGNAL mode_suiveur : STD_LOGIC;
  SIGNAL fin_simu     : BOOLEAN := FALSE;

BEGIN

  u_selection : ENTITY WORK.selection_mode
    PORT MAP (clk => clk, rst_b => rst_b, rx_dv => rx_dv, rx_octet => rx_octet,
              mode_combat => mode_combat, mode_suiveur => mode_suiveur);

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

    -- Simule la réception d'un octet (rx_dv à '1' pendant un cycle).
    PROCEDURE recevoir (octet : STD_LOGIC_VECTOR(7 DOWNTO 0)) IS
    BEGIN
      WAIT UNTIL FALLING_EDGE(clk);
      rx_octet <= octet;
      rx_dv    <= '1';
      WAIT UNTIL FALLING_EDGE(clk);
      rx_dv    <= '0';
      WAIT UNTIL FALLING_EDGE(clk);
    END PROCEDURE recevoir;

  BEGIN
    WAIT FOR 5 us;
    rst_b <= '1';
    WAIT FOR 5 us;
    verifier(mode_combat = '0' AND mode_suiveur = '0', "un mode est actif après le reset");

    recevoir(x"63");   -- 'c'
    verifier(mode_combat = '1' AND mode_suiveur = '0', "'c' ne sélectionne pas le combat");

    recevoir(x"78");   -- 'x' : caractère inconnu, ignoré
    verifier(mode_combat = '1' AND mode_suiveur = '0', "un caractère inconnu change le mode");

    recevoir(x"73");   -- 's'
    verifier(mode_combat = '0' AND mode_suiveur = '1', "'s' ne sélectionne pas le suiveur");

    recevoir(x"63");   -- retour en combat
    verifier(mode_combat = '1' AND mode_suiveur = '0', "passage de suiveur à combat impossible");

    rst_b <= '0';
    WAIT FOR 1 us;
    verifier(mode_combat = '0' AND mode_suiveur = '0', "le reset ne remet pas en attente");

    IF nb_echecs = 0 THEN
      REPORT "TEST OK : tb_selection_mode (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_selection_mode (" & INTEGER'IMAGE(nb_echecs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    WAIT;
  END PROCESS verification;

END ARCHITECTURE tb_selection_mode_arch;
