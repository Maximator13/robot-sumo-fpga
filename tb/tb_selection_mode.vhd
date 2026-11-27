--------------------------------------------------------------------------------
-- Fichier      : tb_selection_mode.vhd
-- Entité       : tb_selection_mode
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant de la sélection du mode par le
--                PC (cahier des charges §1 : 'c' = combat, 's' = suiveur).
-- Principe     : 1) Test unitaire : le banc pilote directement rx_dv et
--                   rx_octet et vérifie le mode après chaque octet :
--                   MODE_ATTENTE après reset, 'c', 's', passage de l'un à
--                   l'autre, 's' directement depuis l'attente, caractères
--                   ignorés (les 254 autres valeurs, dont les majuscules,
--                   testées depuis chacun des trois modes), octet présent
--                   sans rx_dv sans effet, reset asynchrone.
--                2) Test de bout en bout : un « PC » simulé envoie 'c' puis
--                   's' à 9600 bauds sur un vrai UART_RX (52 cycles par bit)
--                   relié à un second exemplaire du module.
--                Chien de garde à 100 ms. "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY tb_selection_mode IS
END ENTITY tb_selection_mode;

ARCHITECTURE tb_selection_mode_arch OF tb_selection_mode IS

  CONSTANT PERIODE_CLK : TIME := 2 us;          -- horloge 500 kHz
  CONSTANT BIT_9600    : TIME := 104_167 ns;    -- bit à 9600 bauds (PC)

  SIGNAL clk      : STD_LOGIC := '0';
  SIGNAL rst_b    : STD_LOGIC := '0';
  SIGNAL fin_simu : BOOLEAN   := FALSE;

  -- Test unitaire.
  SIGNAL rx_dv    : STD_LOGIC := '0';
  SIGNAL rx_octet : STD_LOGIC_VECTOR(7 DOWNTO 0) := (OTHERS => '0');
  SIGNAL mode     : t_mode;

  -- Test de bout en bout (PC -> UART_RX -> selection_mode).
  SIGNAL ligne_pc   : STD_LOGIC := '1';
  SIGNAL uart_dv    : STD_LOGIC;
  SIGNAL uart_octet : STD_LOGIC_VECTOR(7 DOWNTO 0);
  SIGNAL mode_uart  : t_mode;

BEGIN

  u_selection_mode : ENTITY WORK.selection_mode
    PORT MAP (clk => clk, rst_b => rst_b, rx_dv => rx_dv, rx_octet => rx_octet, mode => mode);

  u_UART_RX : ENTITY WORK.UART_RX
    GENERIC MAP (g_CLKS_PER_BIT => UART_CLKS_PER_BIT)
    PORT MAP (i_Clk => clk, i_RX_Serial => ligne_pc, o_RX_DV => uart_dv, o_RX_Byte => uart_octet);

  u_selection_mode_uart : ENTITY WORK.selection_mode
    PORT MAP (clk => clk, rst_b => rst_b, rx_dv => uart_dv, rx_octet => uart_octet, mode => mode_uart);

  horloge : PROCESS
  BEGIN
    WHILE NOT fin_simu LOOP
      clk <= '0'; WAIT FOR PERIODE_CLK / 2;
      clk <= '1'; WAIT FOR PERIODE_CLK / 2;
    END LOOP;
    WAIT;
  END PROCESS horloge;

  chien_de_garde : PROCESS
  BEGIN
    WAIT FOR 100 ms;
    IF NOT fin_simu THEN
      REPORT "TEST KO : tb_selection_mode bloqué (délai de 100 ms dépassé)" SEVERITY FAILURE;
    END IF;
    WAIT;
  END PROCESS chien_de_garde;

  verification : PROCESS
    VARIABLE nb_controles : NATURAL := 0;
    VARIABLE nb_echecs    : NATURAL := 0;
    VARIABLE mode_avant   : t_mode;

    PROCEDURE verifier (condition : BOOLEAN; message : STRING) IS
    BEGIN
      nb_controles := nb_controles + 1;
      IF NOT condition THEN
        nb_echecs := nb_echecs + 1;
      END IF;
      ASSERT condition REPORT "ECHEC : " & message SEVERITY ERROR;
    END PROCEDURE verifier;

    -- Présente un octet avec une impulsion rx_dv d'un cycle (comme UART_RX).
    PROCEDURE recevoir (octet : STD_LOGIC_VECTOR(7 DOWNTO 0)) IS
    BEGIN
      WAIT UNTIL FALLING_EDGE(clk);
      rx_octet <= octet;
      rx_dv    <= '1';
      WAIT UNTIL FALLING_EDGE(clk);
      rx_dv    <= '0';
      WAIT FOR 1 ns;
    END PROCEDURE recevoir;

    -- Le « PC » envoie une trame 8N1 à 9600 bauds.
    PROCEDURE pc_envoie (octet : STD_LOGIC_VECTOR(7 DOWNTO 0)) IS
    BEGIN
      ligne_pc <= '0'; WAIT FOR BIT_9600;
      FOR i IN 0 TO 7 LOOP
        ligne_pc <= octet(i); WAIT FOR BIT_9600;
      END LOOP;
      ligne_pc <= '1'; WAIT FOR BIT_9600;
    END PROCEDURE pc_envoie;

    FUNCTION ascii (c : CHARACTER) RETURN STD_LOGIC_VECTOR IS
    BEGIN
      RETURN STD_LOGIC_VECTOR(TO_UNSIGNED(CHARACTER'POS(c), 8));
    END FUNCTION ascii;

  BEGIN
    ----------------------------------------------------------------------------
    -- 1. Reset : mode d'attente, même si un 'c' arrive pendant le reset.
    ----------------------------------------------------------------------------
    WAIT FOR 1 ns;
    verifier(mode = MODE_ATTENTE, "mode initial différent de MODE_ATTENTE");
    recevoir(CAR_COMBAT);
    verifier(mode = MODE_ATTENTE, "'c' pris en compte pendant le reset");
    rst_b <= '1';
    WAIT FOR 3 * PERIODE_CLK;
    verifier(mode = MODE_ATTENTE, "le mode change seul après le reset");

    ----------------------------------------------------------------------------
    -- 2. Commandes valides et changements de mode.
    ----------------------------------------------------------------------------
    recevoir(CAR_COMBAT);
    verifier(mode = MODE_COMBAT, "'c' ne passe pas en combat");
    recevoir(CAR_COMBAT);
    verifier(mode = MODE_COMBAT, "un second 'c' fait quitter le combat");
    recevoir(CAR_SUIVEUR);
    verifier(mode = MODE_SUIVEUR, "'s' ne passe pas en suiveur");
    recevoir(CAR_COMBAT);
    verifier(mode = MODE_COMBAT, "'c' après 's' ne repasse pas en combat");

    ----------------------------------------------------------------------------
    -- 3. Octet présent sans impulsion rx_dv : aucun effet.
    ----------------------------------------------------------------------------
    rx_octet <= CAR_SUIVEUR;
    WAIT FOR 10 * PERIODE_CLK;
    verifier(mode = MODE_COMBAT, "changement de mode sans rx_dv");

    ----------------------------------------------------------------------------
    -- 4. Les 254 autres octets sont ignorés, depuis chacun des deux modes.
    ----------------------------------------------------------------------------
    FOR depuis IN 0 TO 1 LOOP
      IF depuis = 0 THEN
        recevoir(CAR_COMBAT);
      ELSE
        recevoir(CAR_SUIVEUR);
      END IF;
      mode_avant := mode;
      FOR valeur IN 0 TO 255 LOOP
        IF valeur /= TO_INTEGER(UNSIGNED(CAR_COMBAT)) AND valeur /= TO_INTEGER(UNSIGNED(CAR_SUIVEUR)) THEN
          recevoir(STD_LOGIC_VECTOR(TO_UNSIGNED(valeur, 8)));
          verifier(mode = mode_avant, "l'octet " & INTEGER'IMAGE(valeur) & " a changé le mode");
        END IF;
      END LOOP;
    END LOOP;
    -- En particulier les majuscules ne sont pas des commandes.
    recevoir(ascii('C'));
    verifier(mode = MODE_SUIVEUR, "'C' majuscule pris pour une commande");

    ----------------------------------------------------------------------------
    -- 5. Reset asynchrone : retour immédiat en attente.
    ----------------------------------------------------------------------------
    WAIT FOR 7 ns;
    rst_b <= '0';
    WAIT FOR 1 ns;
    verifier(mode = MODE_ATTENTE, "le reset ne remet pas le mode en attente");
    WAIT FOR 3 * PERIODE_CLK;
    rst_b <= '1';

    ----------------------------------------------------------------------------
    -- 5 bis. Depuis l'attente : les 254 autres octets ne font pas quitter
    --        l'attente, et 's' passe directement en suiveur.
    ----------------------------------------------------------------------------
    FOR valeur IN 0 TO 255 LOOP
      IF valeur /= TO_INTEGER(UNSIGNED(CAR_COMBAT)) AND valeur /= TO_INTEGER(UNSIGNED(CAR_SUIVEUR)) THEN
        recevoir(STD_LOGIC_VECTOR(TO_UNSIGNED(valeur, 8)));
        verifier(mode = MODE_ATTENTE, "en attente, l'octet " & INTEGER'IMAGE(valeur) & " a changé le mode");
      END IF;
    END LOOP;
    recevoir(CAR_SUIVEUR);
    verifier(mode = MODE_SUIVEUR, "'s' depuis l'attente ne passe pas en suiveur");
    rst_b <= '0';
    WAIT FOR 3 * PERIODE_CLK;
    rst_b <= '1';

    ----------------------------------------------------------------------------
    -- 6. Bout en bout : 'c' puis 's' envoyés par le PC à 9600 bauds.
    ----------------------------------------------------------------------------
    WAIT FOR 5 * PERIODE_CLK;
    verifier(mode_uart = MODE_ATTENTE, "bout en bout : mode initial incorrect");
    pc_envoie(ascii('c'));
    WAIT FOR 4 * PERIODE_CLK;
    verifier(mode_uart = MODE_COMBAT, "bout en bout : 'c' envoyé par le PC ne passe pas en combat");
    pc_envoie(ascii('s'));
    WAIT FOR 4 * PERIODE_CLK;
    verifier(mode_uart = MODE_SUIVEUR, "bout en bout : 's' envoyé par le PC ne passe pas en suiveur");

    -- Bilan.
    IF nb_echecs = 0 THEN
      REPORT "TEST OK : tb_selection_mode (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_selection_mode (" & INTEGER'IMAGE(nb_echecs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    std.env.finish;
  END PROCESS verification;

END ARCHITECTURE tb_selection_mode_arch;
