--------------------------------------------------------------------------------
-- Fichier      : tb_UART_RX.vhd
-- Entité       : tb_UART_RX
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant de la liaison série : émetteur
--                UART_TX et récepteur UART_RX fournis par l'enseignant,
--                réglés comme dans le robot (horloge 500 kHz,
--                g_CLKS_PER_BIT = 52, soit 9600 bauds 8N1).
-- Principe     : 1) Format de trame de UART_TX vérifié par un récepteur de
--                   référence écrit dans le banc (mesure en temps simulé :
--                   bit de 104 µs, start à '0', données poids faible en
--                   premier, stop à '1'), et durée des signaux de contrôle.
--                2) Bouclage TX -> RX des 256 valeurs d'octet : chaque octet
--                   est reçu identique, avec une seule impulsion o_RX_DV
--                   d'un cycle par octet.
--                3) Réception d'une trame produite par un « PC » idéal à
--                   9600 bauds exacts, puis avec un débit décalé de -2 % et
--                   +2 %, et de deux trames collées (sans temps de repos).
--                4) Une impulsion parasite plus courte qu'un demi-bit sur la
--                   ligne RX ne produit aucun octet.
--                Chien de garde à 1 s. "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY tb_UART_RX IS
END ENTITY tb_UART_RX;

ARCHITECTURE tb_UART_RX_arch OF tb_UART_RX IS

  CONSTANT PERIODE_CLK : TIME := 2 us;                          -- horloge système 500 kHz
  CONSTANT DUREE_BIT   : TIME := UART_CLKS_PER_BIT * PERIODE_CLK;  -- 104 µs (débit du robot)
  CONSTANT BIT_9600    : TIME := 104_167 ns;                    -- bit à 9600 bauds exacts (PC)

  SIGNAL clk       : STD_LOGIC := '0';
  SIGNAL fin_simu  : BOOLEAN   := FALSE;

  -- Émetteur.
  SIGNAL tx_dv     : STD_LOGIC := '0';
  SIGNAL tx_octet  : STD_LOGIC_VECTOR(7 DOWNTO 0) := (OTHERS => '0');
  SIGNAL tx_actif  : STD_LOGIC;
  SIGNAL tx_ligne  : STD_LOGIC;
  SIGNAL tx_fini   : STD_LOGIC;

  -- Récepteur : son entrée est soit la sortie de l'émetteur (bouclage), soit
  -- une ligne pilotée directement par le banc (PC simulé).
  SIGNAL bouclage  : BOOLEAN   := TRUE;
  SIGNAL ligne_pc  : STD_LOGIC := '1';
  SIGNAL rx_entree : STD_LOGIC;
  SIGNAL rx_dv     : STD_LOGIC;
  SIGNAL rx_octet  : STD_LOGIC_VECTOR(7 DOWNTO 0);

  -- Surveillance des impulsions o_RX_DV (nombre, largeur).
  SIGNAL nb_rx_dv        : NATURAL := 0;
  SIGNAL echecs_largeur  : NATURAL := 0;

BEGIN

  u_UART_TX : ENTITY WORK.UART_TX
    GENERIC MAP (g_CLKS_PER_BIT => UART_CLKS_PER_BIT)
    PORT MAP (i_Clk => clk, i_TX_DV => tx_dv, i_TX_Byte => tx_octet,
              o_TX_Active => tx_actif, o_TX_Serial => tx_ligne, o_TX_Done => tx_fini);

  u_UART_RX : ENTITY WORK.UART_RX
    GENERIC MAP (g_CLKS_PER_BIT => UART_CLKS_PER_BIT)
    PORT MAP (i_Clk => clk, i_RX_Serial => rx_entree, o_RX_DV => rx_dv, o_RX_Byte => rx_octet);

  rx_entree <= tx_ligne WHEN bouclage ELSE ligne_pc;

  horloge : PROCESS
  BEGIN
    WHILE NOT fin_simu LOOP
      clk <= '0'; WAIT FOR PERIODE_CLK / 2;
      clk <= '1'; WAIT FOR PERIODE_CLK / 2;
    END LOOP;
    WAIT;
  END PROCESS horloge;

  -- Compte les impulsions o_RX_DV et vérifie qu'elles durent un seul cycle.
  surveillance_rx_dv : PROCESS (clk)
    VARIABLE largeur : NATURAL := 0;
  BEGIN
    IF RISING_EDGE(clk) THEN
      IF rx_dv = '1' THEN
        largeur := largeur + 1;
        IF largeur = 1 THEN
          nb_rx_dv <= nb_rx_dv + 1;
        ELSIF largeur = 2 THEN
          echecs_largeur <= echecs_largeur + 1;
          REPORT "ECHEC : o_RX_DV reste à '1' plus d'un cycle" SEVERITY ERROR;
        END IF;
      ELSE
        largeur := 0;
      END IF;
    END IF;
  END PROCESS surveillance_rx_dv;

  chien_de_garde : PROCESS
  BEGIN
    WAIT FOR 1 sec;
    IF NOT fin_simu THEN
      REPORT "TEST KO : tb_UART_RX bloqué (délai de 1 s dépassé)" SEVERITY FAILURE;
    END IF;
    WAIT;
  END PROCESS chien_de_garde;

  verification : PROCESS
    VARIABLE nb_controles : NATURAL := 0;
    VARIABLE nb_echecs    : NATURAL := 0;

    PROCEDURE verifier (condition : BOOLEAN; message : STRING) IS
    BEGIN
      nb_controles := nb_controles + 1;
      IF NOT condition THEN
        nb_echecs := nb_echecs + 1;
      END IF;
      ASSERT condition REPORT "ECHEC : " & message SEVERITY ERROR;
    END PROCEDURE verifier;

    FUNCTION hex (v : STD_LOGIC_VECTOR(7 DOWNTO 0)) RETURN STRING IS
    BEGIN
      RETURN INTEGER'IMAGE(TO_INTEGER(UNSIGNED(v)));
    END FUNCTION hex;

    -- Demande l'envoi d'un octet à UART_TX (impulsion i_TX_DV d'un cycle,
    -- émetteur au repos).
    PROCEDURE emettre (octet : STD_LOGIC_VECTOR(7 DOWNTO 0)) IS
    BEGIN
      IF tx_actif = '1' THEN
        WAIT UNTIL tx_actif = '0';
      END IF;
      WAIT UNTIL RISING_EDGE(clk);
      tx_octet <= octet;
      tx_dv    <= '1';
      WAIT UNTIL RISING_EDGE(clk);
      tx_dv    <= '0';
    END PROCEDURE emettre;

    -- Le « PC » envoie une trame 8N1 sur ligne_pc avec une durée de bit donnée.
    PROCEDURE pc_envoie (octet : STD_LOGIC_VECTOR(7 DOWNTO 0); bit_t : TIME) IS
    BEGIN
      ligne_pc <= '0'; WAIT FOR bit_t;                 -- bit de start
      FOR i IN 0 TO 7 LOOP
        ligne_pc <= octet(i); WAIT FOR bit_t;          -- données, poids faible en premier
      END LOOP;
      ligne_pc <= '1'; WAIT FOR bit_t;                 -- bit de stop
    END PROCEDURE pc_envoie;

    -- Attend un octet sur la sortie du récepteur (au plus 3 trames) et le
    -- compare à la valeur attendue.
    PROCEDURE attendre_reception (attendu : STD_LOGIC_VECTOR(7 DOWNTO 0); contexte : STRING) IS
    BEGIN
      IF rx_dv /= '1' THEN
        WAIT UNTIL rx_dv = '1' FOR 30 * DUREE_BIT;
      END IF;
      verifier(rx_dv = '1', contexte & " : aucun octet reçu");
      verifier(rx_octet = attendu, contexte & " : reçu " & hex(rx_octet) & " au lieu de " & hex(attendu));
      WAIT UNTIL RISING_EDGE(clk);
    END PROCEDURE attendre_reception;

    -- Le « PC » envoie une trame puis on vérifie qu'exactement un octet a
    -- été reçu, avec la bonne valeur. o_RX_DV (1 cycle) tombe au milieu du
    -- bit de stop, donc avant la fin de pc_envoie : on s'appuie sur le
    -- compteur d'impulsions et sur o_RX_Byte, lu juste après l'impulsion
    -- (avant le premier bit de données d'une éventuelle trame suivante).
    PROCEDURE pc_envoie_et_verifie (octet : STD_LOGIC_VECTOR(7 DOWNTO 0); bit_t : TIME; contexte : STRING) IS
      VARIABLE nb_avant : NATURAL;
    BEGIN
      nb_avant := nb_rx_dv;
      pc_envoie(octet, bit_t);
      WAIT FOR 2 * PERIODE_CLK;
      verifier(nb_rx_dv = nb_avant + 1, contexte & " : " & INTEGER'IMAGE(nb_rx_dv - nb_avant) & " octet(s) reçu(s) au lieu de 1");
      verifier(rx_octet = octet, contexte & " : reçu " & hex(rx_octet) & " au lieu de " & hex(octet));
    END PROCEDURE pc_envoie_et_verifie;

    VARIABLE octet_lu : STD_LOGIC_VECTOR(7 DOWNTO 0);
    VARIABLE t_debut  : TIME;
    VARIABLE nb_dv_avant : NATURAL;
    VARIABLE duree_fini  : NATURAL;

  BEGIN
    ----------------------------------------------------------------------------
    -- 1. Repos : après le premier front, ligne TX à '1', émetteur inactif.
    ----------------------------------------------------------------------------
    WAIT UNTIL RISING_EDGE(clk);
    WAIT FOR 1 ns;
    verifier(tx_ligne = '1', "ligne TX pas au repos ('1') après le premier front");
    verifier(tx_actif = '0' AND tx_fini = '0', "émetteur actif au repos");

    ----------------------------------------------------------------------------
    -- 2. Format d'une trame 'c' (0x63 = 0110 0011, bit 0 = 1 envoyé en premier).
    ----------------------------------------------------------------------------
    emettre(CAR_COMBAT);
    WAIT UNTIL FALLING_EDGE(tx_ligne);
    t_debut := NOW;
    WAIT UNTIL RISING_EDGE(tx_ligne);       -- fin du start (le bit 0 vaut 1)
    verifier(NOW - t_debut = DUREE_BIT,
             "bit de start de " & TIME'IMAGE(NOW - t_debut) & " au lieu de 104 us");
    verifier(tx_actif = '1', "o_TX_Active à '0' pendant la trame");
    -- Lecture complète de la même trame par le récepteur de référence.
    WAIT FOR DUREE_BIT / 2;                 -- milieu du bit 0
    octet_lu(0) := tx_ligne;
    FOR i IN 1 TO 7 LOOP
      WAIT FOR DUREE_BIT;
      octet_lu(i) := tx_ligne;
    END LOOP;
    WAIT FOR DUREE_BIT;
    verifier(tx_ligne = '1', "bit de stop absent");
    verifier(octet_lu = CAR_COMBAT, "trame TX décodée " & hex(octet_lu) & " au lieu de 'c'");
    -- Fin de trame : o_TX_Done à '1' pendant 2 cycles, puis retour au repos.
    WAIT UNTIL tx_fini = '1';
    duree_fini := 0;
    WHILE tx_fini = '1' LOOP
      WAIT UNTIL RISING_EDGE(clk);
      WAIT FOR 1 ns;
      duree_fini := duree_fini + 1;
    END LOOP;
    verifier(duree_fini = 2, "o_TX_Done à '1' pendant " & INTEGER'IMAGE(duree_fini) & " cycles au lieu de 2");
    verifier(tx_actif = '0', "o_TX_Active encore à '1' après la trame");
    verifier(NOW - t_debut >= 10 * DUREE_BIT, "trame plus courte que 10 bits");
    -- Le récepteur bouclé a reçu le même octet.
    verifier(rx_octet = CAR_COMBAT, "bouclage : 'c' non reçu par UART_RX");

    ----------------------------------------------------------------------------
    -- 3. Bouclage des 256 valeurs d'octet, trames enchaînées.
    ----------------------------------------------------------------------------
    nb_dv_avant := nb_rx_dv;
    FOR valeur IN 0 TO 255 LOOP
      emettre(STD_LOGIC_VECTOR(TO_UNSIGNED(valeur, 8)));
      attendre_reception(STD_LOGIC_VECTOR(TO_UNSIGNED(valeur, 8)), "bouclage valeur " & INTEGER'IMAGE(valeur));
    END LOOP;
    WAIT FOR 2 * DUREE_BIT;
    verifier(nb_rx_dv - nb_dv_avant = 256,
             INTEGER'IMAGE(nb_rx_dv - nb_dv_avant) & " impulsions o_RX_DV pour 256 octets");

    ----------------------------------------------------------------------------
    -- 4. Réception depuis un PC à 9600 bauds exacts, puis à -2 % et +2 %.
    ----------------------------------------------------------------------------
    bouclage <= FALSE;
    WAIT FOR 2 * DUREE_BIT;
    pc_envoie_et_verifie(CAR_COMBAT, BIT_9600, "PC 9600 'c'");
    pc_envoie_et_verifie(CAR_SUIVEUR, BIT_9600, "PC 9600 's'");
    pc_envoie_et_verifie(x"00", BIT_9600, "PC 9600 0x00");
    pc_envoie_et_verifie(x"FF", BIT_9600, "PC 9600 0xFF");
    pc_envoie_et_verifie(x"55", BIT_9600 * 102 / 100, "PC lent -2 % 0x55");
    pc_envoie_et_verifie(x"AA", BIT_9600 * 102 / 100, "PC lent -2 % 0xAA");
    pc_envoie_et_verifie(x"55", BIT_9600 * 98 / 100, "PC rapide +2 % 0x55");
    pc_envoie_et_verifie(x"AA", BIT_9600 * 98 / 100, "PC rapide +2 % 0xAA");

    ----------------------------------------------------------------------------
    -- 5. Deux trames collées (le start suit immédiatement le stop) : le
    --    premier octet est lu pendant la seconde trame, le second à la fin.
    ----------------------------------------------------------------------------
    nb_dv_avant := nb_rx_dv;
    pc_envoie(CAR_COMBAT, BIT_9600);
    verifier(nb_rx_dv = nb_dv_avant + 1 AND rx_octet = CAR_COMBAT, "trames collées : 1er octet 'c' non reçu");
    pc_envoie(CAR_SUIVEUR, BIT_9600);
    WAIT FOR 2 * PERIODE_CLK;
    verifier(nb_rx_dv - nb_dv_avant = 2, "trames collées : " & INTEGER'IMAGE(nb_rx_dv - nb_dv_avant) & " octet(s) reçu(s) au lieu de 2");
    verifier(rx_octet = CAR_SUIVEUR, "trames collées : 2e octet 's' non reçu");

    ----------------------------------------------------------------------------
    -- 6. Parasite de 20 µs (moins d'un demi-bit) : aucun octet ne doit sortir.
    ----------------------------------------------------------------------------
    WAIT FOR 5 * DUREE_BIT;
    nb_dv_avant := nb_rx_dv;
    ligne_pc <= '0'; WAIT FOR 20 us; ligne_pc <= '1';
    WAIT FOR 20 * DUREE_BIT;
    verifier(nb_rx_dv = nb_dv_avant, "une impulsion de 20 us a été prise pour un octet");
    -- Le récepteur fonctionne toujours après le parasite.
    pc_envoie_et_verifie(CAR_SUIVEUR, BIT_9600, "après parasite");

    -- Bilan.
    WAIT FOR 1 us;
    IF nb_echecs = 0 AND echecs_largeur = 0 THEN
      REPORT "TEST OK : tb_UART_RX (" & INTEGER'IMAGE(nb_controles) & " controles, bouclage 256 octets)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_UART_RX (" & INTEGER'IMAGE(nb_echecs + echecs_largeur) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    std.env.finish;
  END PROCESS verification;

END ARCHITECTURE tb_UART_RX_arch;
