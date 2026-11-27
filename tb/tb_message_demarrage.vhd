--------------------------------------------------------------------------------
-- Fichier      : tb_message_demarrage.vhd
-- Entité       : tb_message_demarrage
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant de l'envoi du message de
--                démarrage, avec l'émetteur série UART_TX.
-- Principe     : Horloge 500 kHz, 52 cycles par bit (9600 bauds). Un
--                processus "pc" lit la ligne série comme le ferait le PC
--                (lecture de chaque bit en son milieu) et range les
--                caractères reçus. On vérifie que le message reçu est
--                exactement "Robot PRET !" + LF + CR, envoyé une seule
--                fois, puis renvoyé après un nouveau reset.
--                "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY tb_message_demarrage IS
END ENTITY tb_message_demarrage;

ARCHITECTURE tb_message_demarrage_arch OF tb_message_demarrage IS

  CONSTANT CLKS_PER_BIT : INTEGER := 52;
  CONSTANT DUREE_BIT    : TIME    := 104 us;   -- 52 cycles de 2 µs

  -- Message attendu : "Robot PRET !" puis LF (10) et CR (13).
  CONSTANT ATTENDU : STRING := "Robot PRET !" & CHARACTER'VAL(10) & CHARACTER'VAL(13);

  SIGNAL clk      : STD_LOGIC := '0';
  SIGNAL rst_b    : STD_LOGIC := '0';
  SIGNAL tx_dv    : STD_LOGIC;
  SIGNAL tx_octet : STD_LOGIC_VECTOR(7 DOWNTO 0);
  SIGNAL tx_actif : STD_LOGIC;
  SIGNAL ligne    : STD_LOGIC;
  SIGNAL fin_simu : BOOLEAN := FALSE;

  -- Caractères reçus par le "PC".
  SIGNAL recu     : STRING(1 TO 40) := (OTHERS => ' ');
  SIGNAL nb_recus : NATURAL := 0;

BEGIN

  u_message : ENTITY WORK.message_demarrage
    PORT MAP (clk => clk, rst_b => rst_b, tx_actif => tx_actif,
              tx_dv => tx_dv, tx_octet => tx_octet);

  u_tx : ENTITY WORK.UART_TX
    GENERIC MAP (g_CLKS_PER_BIT => CLKS_PER_BIT)
    PORT MAP (i_Clk => clk, i_TX_DV => tx_dv, i_TX_Byte => tx_octet,
              o_TX_Active => tx_actif, o_TX_Serial => ligne, o_TX_Done => OPEN);

  horloge : PROCESS
  BEGIN
    WHILE NOT fin_simu LOOP
      clk <= '0'; WAIT FOR 1 us;
      clk <= '1'; WAIT FOR 1 us;
    END LOOP;
    WAIT;
  END PROCESS horloge;

  -- Le "PC" : détecte le bit de start, lit les 8 bits au milieu de chacun.
  pc : PROCESS
    VARIABLE octet : STD_LOGIC_VECTOR(7 DOWNTO 0);
  BEGIN
    WAIT UNTIL ligne = '1';
    LOOP
      WAIT UNTIL FALLING_EDGE(ligne);
      WAIT FOR DUREE_BIT / 2;            -- milieu du bit de start
      FOR i IN 0 TO 7 LOOP
        WAIT FOR DUREE_BIT;
        octet(i) := ligne;               -- bits de poids faible en premier
      END LOOP;
      WAIT FOR DUREE_BIT;                -- bit de stop
      IF nb_recus < 40 THEN
        recu(nb_recus + 1) <= CHARACTER'VAL(TO_INTEGER(UNSIGNED(octet)));
        nb_recus <= nb_recus + 1;
      END IF;
    END LOOP;
  END PROCESS pc;

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

  BEGIN
    -- Mise sous tension : 14 caractères doivent arriver, une seule fois.
    WAIT FOR 20 us;
    rst_b <= '1';
    WAIT FOR 20 ms;
    verifier(nb_recus = 14, INTEGER'IMAGE(nb_recus) & " caractères reçus au lieu de 14");
    verifier(recu(1 TO 14) = ATTENDU, "message reçu incorrect : " & recu(1 TO 12));

    -- Nouveau reset : le message est renvoyé.
    rst_b <= '0';
    WAIT FOR 20 us;
    rst_b <= '1';
    WAIT FOR 20 ms;
    verifier(nb_recus = 28, "message non renvoyé après le reset");
    verifier(recu(15 TO 28) = ATTENDU, "second message incorrect : " & recu(15 TO 26));

    IF nb_echecs = 0 THEN
      REPORT "TEST OK : tb_message_demarrage (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_message_demarrage (" & INTEGER'IMAGE(nb_echecs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    WAIT;
  END PROCESS verification;

END ARCHITECTURE tb_message_demarrage_arch;
