--------------------------------------------------------------------------------
-- Fichier      : tb_UART_RX.vhd
-- Entité       : tb_UART_RX
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant des modules série fournis
--                UART_TX et UART_RX, reliés en boucle (la sortie de
--                l'émetteur entre dans le récepteur).
-- Principe     : Horloge 500 kHz, 52 cycles par bit (9600 bauds, comme dans
--                le robot). On envoie 'c', 's', x"00" et x"FF" et on vérifie
--                que chaque octet est reçu à l'identique, puis que la durée
--                d'une trame vaut 10 bits (start, 8 données, stop).
--                "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY tb_UART_RX IS
END ENTITY tb_UART_RX;

ARCHITECTURE tb_UART_RX_arch OF tb_UART_RX IS

  CONSTANT CLKS_PER_BIT : INTEGER := 52;   -- 500 kHz / 9600 bauds

  SIGNAL clk       : STD_LOGIC := '0';
  SIGNAL tx_dv     : STD_LOGIC := '0';
  SIGNAL tx_octet  : STD_LOGIC_VECTOR(7 DOWNTO 0) := (OTHERS => '0');
  SIGNAL tx_actif  : STD_LOGIC;
  SIGNAL ligne     : STD_LOGIC;
  SIGNAL tx_fini   : STD_LOGIC;
  SIGNAL rx_dv     : STD_LOGIC;
  SIGNAL rx_octet  : STD_LOGIC_VECTOR(7 DOWNTO 0);
  SIGNAL fin_simu  : BOOLEAN := FALSE;

BEGIN

  u_tx : ENTITY WORK.UART_TX
    GENERIC MAP (g_CLKS_PER_BIT => CLKS_PER_BIT)
    PORT MAP (i_Clk => clk, i_TX_DV => tx_dv, i_TX_Byte => tx_octet,
              o_TX_Active => tx_actif, o_TX_Serial => ligne, o_TX_Done => tx_fini);

  u_rx : ENTITY WORK.UART_RX
    GENERIC MAP (g_CLKS_PER_BIT => CLKS_PER_BIT)
    PORT MAP (i_Clk => clk, i_RX_Serial => ligne, o_RX_DV => rx_dv, o_RX_Byte => rx_octet);

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
    VARIABLE t_debut      : TIME;

    PROCEDURE verifier (condition : BOOLEAN; message : STRING) IS
    BEGIN
      nb_controles := nb_controles + 1;
      IF NOT condition THEN
        nb_echecs := nb_echecs + 1;
        REPORT "ECHEC : " & message SEVERITY ERROR;
      END IF;
    END PROCEDURE verifier;

    -- Envoie un octet par l'émetteur et vérifie sa réception.
    PROCEDURE boucler (octet : STD_LOGIC_VECTOR(7 DOWNTO 0)) IS
    BEGIN
      WAIT UNTIL FALLING_EDGE(clk);
      tx_octet <= octet;
      tx_dv    <= '1';
      WAIT UNTIL FALLING_EDGE(clk);
      tx_dv    <= '0';
      WAIT UNTIL rx_dv = '1' FOR 2 ms;
      verifier(rx_dv = '1', "octet non reçu");
      verifier(rx_octet = octet, "octet reçu différent de l'octet envoyé");
      WAIT UNTIL tx_actif = '0' FOR 2 ms;
      WAIT FOR 20 us;
    END PROCEDURE boucler;

  BEGIN
    WAIT FOR 20 us;
    boucler(x"63");   -- 'c'
    boucler(x"73");   -- 's'
    boucler(x"00");
    boucler(x"FF");

    -- Durée d'une trame : 10 bits de 52 cycles de 2 µs = 1040 µs.
    WAIT UNTIL FALLING_EDGE(clk);
    tx_octet <= x"55";
    tx_dv    <= '1';
    WAIT UNTIL FALLING_EDGE(ligne);
    t_debut := NOW;
    tx_dv    <= '0';
    WAIT UNTIL tx_fini = '1' FOR 2 ms;
    verifier(NOW - t_debut >= 1030 us AND NOW - t_debut <= 1050 us,
             "durée de trame de " & TIME'IMAGE(NOW - t_debut));

    IF nb_echecs = 0 THEN
      REPORT "TEST OK : tb_UART_RX (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_UART_RX (" & INTEGER'IMAGE(nb_echecs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    WAIT;
  END PROCESS verification;

END ARCHITECTURE tb_UART_RX_arch;
