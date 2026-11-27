--------------------------------------------------------------------------------
-- Fichier      : tb_message_demarrage.vhd
-- Entité       : tb_message_demarrage
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant de l'envoi du message de
--                démarrage "Robot PRET !\n\r" (cahier des charges §1), avec
--                le vrai émetteur UART_TX réglé comme dans le robot
--                (500 kHz, 52 cycles par bit, 9600 bauds 8N1).
-- Principe     : L'horloge reproduit celle du diviseur de fréquence sur la
--                carte : figée à '0' pendant tout le reset (1 ms, comme
--                circuit_reset), premier front montant 1 µs après le
--                relâchement. UART_TX, qui n'a pas de reset, est donc figé
--                pendant le reset, exactement comme dans le robot.
--                Un récepteur de référence décode la ligne série (milieu de
--                bit, start et stop contrôlés) et journalise chaque trame :
--                octet, validité, instant de début.
--                Un observateur vérifie que chaque demande tx_dv est bien
--                prise par UART_TX (son registre interne r_TX_Data, lu par
--                nom externe, contient l'octet demandé et la trame démarre).
--                Scénarios : mise sous tension ; silence après le message ;
--                reset dans le cycle qui suit une demande prise par UART_TX
--                (o_TX_Active encore à '0') ; reset au milieu d'une trame.
--                Après chaque reset : les 14 dernières trames sont valides et
--                forment exactement le message, précédées d'au moins une
--                trame de silence (garde) pour que le PC se resynchronise.
--                Chien de garde à 300 ms. "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY tb_message_demarrage IS
END ENTITY tb_message_demarrage;

ARCHITECTURE tb_message_demarrage_arch OF tb_message_demarrage IS

  CONSTANT PERIODE_CLK : TIME := 2 us;                            -- horloge 500 kHz
  CONSTANT DUREE_BIT   : TIME := UART_CLKS_PER_BIT * PERIODE_CLK;  -- 104 µs
  CONSTANT DUREE_TRAME : TIME := 10 * DUREE_BIT;                   -- start + 8 + stop
  CONSTANT DUREE_RESET : TIME := 1 ms;                             -- comme circuit_reset
  CONSTANT MESSAGE_ATTENDU : STRING := "Robot PRET !" & LF & CR;   -- 14 caractères

  SIGNAL clk      : STD_LOGIC := '0';
  SIGNAL rst_b    : STD_LOGIC := '0';
  SIGNAL fin_simu : BOOLEAN   := FALSE;

  SIGNAL tx_dv    : STD_LOGIC;
  SIGNAL tx_octet : STD_LOGIC_VECTOR(7 DOWNTO 0);
  SIGNAL tx_actif : STD_LOGIC;
  SIGNAL tx_ligne : STD_LOGIC;
  SIGNAL tx_fini  : STD_LOGIC;
  SIGNAL envoye   : STD_LOGIC;

  -- Journal du récepteur de référence.
  CONSTANT MAX_TRAMES : NATURAL := 128;
  TYPE t_octets   IS ARRAY (0 TO MAX_TRAMES - 1) OF STD_LOGIC_VECTOR(7 DOWNTO 0);
  TYPE t_instants IS ARRAY (0 TO MAX_TRAMES - 1) OF TIME;
  TYPE t_validites IS ARRAY (0 TO MAX_TRAMES - 1) OF BOOLEAN;
  SIGNAL octets_recus  : t_octets;
  SIGNAL debuts_trames : t_instants;
  SIGNAL trames_valides : t_validites;
  SIGNAL repos_avant   : t_instants;     -- durée de ligne au repos ('1') avant chaque start
  SIGNAL nb_recus      : NATURAL := 0;
  SIGNAL instant_repos : TIME := 0 ns;   -- dernier passage de la ligne à '1'

  -- Observateurs.
  SIGNAL nb_tx_dv      : NATURAL := 0;
  SIGNAL erreurs_dv    : NATURAL := 0;   -- tx_dv de plus d'un cycle
  SIGNAL erreurs_prise : NATURAL := 0;   -- demande non prise par UART_TX

  FUNCTION ascii (c : CHARACTER) RETURN STD_LOGIC_VECTOR IS
  BEGIN
    RETURN STD_LOGIC_VECTOR(TO_UNSIGNED(CHARACTER'POS(c), 8));
  END FUNCTION ascii;

BEGIN

  u_message_demarrage : ENTITY WORK.message_demarrage
    PORT MAP (clk => clk, rst_b => rst_b, tx_actif => tx_actif, tx_fini => tx_fini,
              tx_dv => tx_dv, tx_octet => tx_octet, envoye => envoye);

  u_UART_TX : ENTITY WORK.UART_TX
    GENERIC MAP (g_CLKS_PER_BIT => UART_CLKS_PER_BIT)
    PORT MAP (i_Clk => clk, i_TX_DV => tx_dv, i_TX_Byte => tx_octet,
              o_TX_Active => tx_actif, o_TX_Serial => tx_ligne, o_TX_Done => tx_fini);

  -- Horloge 500 kHz comme en sortie du diviseur de fréquence : '0' pendant le
  -- reset (y compris si le reset arrive pendant la demi-période haute),
  -- premier front montant une demi-période après le relâchement.
  horloge : PROCESS
  BEGIN
    WHILE NOT fin_simu LOOP
      IF rst_b = '0' THEN
        clk <= '0';
        WAIT UNTIL rst_b = '1';
        WAIT FOR PERIODE_CLK / 2;
      END IF;
      clk <= '1';
      WAIT UNTIL rst_b = '0' FOR PERIODE_CLK / 2;
      clk <= '0';
      IF rst_b = '1' THEN
        WAIT UNTIL rst_b = '0' FOR PERIODE_CLK / 2;
      END IF;
    END LOOP;
    WAIT;
  END PROCESS horloge;

  -- Mémorise l'instant où la ligne TX passe (ou revient) à '1'.
  suivi_repos : PROCESS (tx_ligne)
  BEGIN
    IF tx_ligne = '1' THEN
      instant_repos <= NOW;
    END IF;
  END PROCESS suivi_repos;

  -- Récepteur de référence : décode et journalise chaque trame de la ligne TX
  -- (y compris les trames abîmées par un reset, marquées non valides), avec
  -- la durée pendant laquelle la ligne était au repos avant son start.
  recepteur_ref : PROCESS
    VARIABLE v      : STD_LOGIC_VECTOR(7 DOWNTO 0);
    VARIABLE t      : TIME;
    VARIABLE valide : BOOLEAN;
  BEGIN
    WAIT UNTIL FALLING_EDGE(tx_ligne);
    t := NOW;
    IF nb_recus < MAX_TRAMES THEN
      repos_avant(nb_recus) <= t - instant_repos;
    END IF;
    WAIT FOR DUREE_BIT / 2;                      -- milieu du bit de start
    valide := (tx_ligne = '0');
    IF valide THEN
      FOR i IN 0 TO 7 LOOP
        WAIT FOR DUREE_BIT;                      -- milieu du bit de données i
        v(i) := tx_ligne;
      END LOOP;
      WAIT FOR DUREE_BIT;                        -- milieu du bit de stop
      valide := (tx_ligne = '1');
    ELSE
      v := (OTHERS => 'X');
    END IF;
    IF nb_recus < MAX_TRAMES THEN
      octets_recus(nb_recus)   <= v;
      debuts_trames(nb_recus)  <= t;
      trames_valides(nb_recus) <= valide;
    END IF;
    nb_recus <= nb_recus + 1;
    -- Si le stop est à '0' (ligne figée basse), attendre le retour au repos.
    IF tx_ligne = '0' THEN
      WAIT UNTIL tx_ligne = '1';
    END IF;
  END PROCESS recepteur_ref;

  -- Chaque tx_dv dure un seul cycle.
  observateur_dv : PROCESS (clk)
    VARIABLE largeur : NATURAL := 0;
  BEGIN
    IF RISING_EDGE(clk) THEN
      IF tx_dv = '1' THEN
        largeur := largeur + 1;
        IF largeur = 1 THEN
          nb_tx_dv <= nb_tx_dv + 1;
        ELSE
          erreurs_dv <= erreurs_dv + 1;
          REPORT "ECHEC : tx_dv reste à '1' plus d'un cycle" SEVERITY ERROR;
        END IF;
      ELSE
        largeur := 0;
      END IF;
    END IF;
  END PROCESS observateur_dv;

  -- Chaque demande doit être prise par UART_TX : au front où UART_TX lit
  -- tx_dv, il copie l'octet dans r_TX_Data et démarre la trame (o_TX_Active
  -- à '1' au front suivant). Une demande faite hors repos serait ignorée.
  observateur_prise : PROCESS
    ALIAS donnee_uart IS << SIGNAL .tb_message_demarrage.u_UART_TX.r_TX_Data : STD_LOGIC_VECTOR(7 DOWNTO 0) >>;
    VARIABLE octet_demande : STD_LOGIC_VECTOR(7 DOWNTO 0);
  BEGIN
    WAIT UNTIL RISING_EDGE(clk) AND tx_dv = '1';   -- front de lecture par UART_TX
    octet_demande := tx_octet;
    WAIT FOR 1 ns;
    IF donnee_uart /= octet_demande THEN
      erreurs_prise <= erreurs_prise + 1;
      REPORT "ECHEC : demande d'envoi ignorée par UART_TX (octet " &
             INTEGER'IMAGE(TO_INTEGER(UNSIGNED(octet_demande))) & " perdu)" SEVERITY ERROR;
    END IF;
    WAIT UNTIL RISING_EDGE(clk) OR rst_b = '0';
    WAIT FOR 1 ns;
    IF rst_b = '1' AND tx_actif /= '1' THEN
      erreurs_prise <= erreurs_prise + 1;
      REPORT "ECHEC : la trame ne démarre pas après la demande" SEVERITY ERROR;
    END IF;
  END PROCESS observateur_prise;

  chien_de_garde : PROCESS
  BEGIN
    WAIT FOR 300 ms;
    IF NOT fin_simu THEN
      REPORT "TEST KO : tb_message_demarrage bloqué (délai de 300 ms dépassé)" SEVERITY FAILURE;
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

    -- Après un relâchement du reset à l'instant t_release : attend la fin du
    -- message puis vérifie que les 14 dernières trames sont valides, forment
    -- exactement le message, commencent après le relâchement, et sont
    -- précédées d'au moins une durée de trame de silence (garde).
    PROCEDURE verifier_message_apres_reset (t_release : TIME; contexte : STRING) IS
      VARIABLE premier : INTEGER;
    BEGIN
      WAIT UNTIL envoye = '1' FOR 40 ms;
      verifier(envoye = '1', contexte & " : message non terminé");
      WAIT FOR DUREE_BIT;
      premier := nb_recus - MESSAGE_ATTENDU'LENGTH;
      verifier(premier >= 0, contexte & " : moins de 14 trames reçues");
      IF premier >= 0 THEN
        FOR i IN 0 TO MESSAGE_ATTENDU'LENGTH - 1 LOOP
          verifier(trames_valides(premier + i), contexte & " : trame " & INTEGER'IMAGE(i) & " du message mal formée");
          verifier(octets_recus(premier + i) = ascii(MESSAGE_ATTENDU(i + 1)),
                   contexte & " : caractère " & INTEGER'IMAGE(i) & " du message incorrect");
        END LOOP;
        verifier(debuts_trames(premier) > t_release, contexte & " : le message a commencé avant la fin du reset");
        -- Garde : la ligne doit être restée au repos au moins une durée de
        -- trame avant le start du 'R', pour que le PC se resynchronise.
        verifier(repos_avant(premier) >= DUREE_TRAME,
                 contexte & " : ligne au repos seulement " & TIME'IMAGE(repos_avant(premier)) &
                 " avant le message (une trame minimum)");
      END IF;
    END PROCEDURE verifier_message_apres_reset;

    VARIABLE t_release : TIME;
    VARIABLE nb_avant  : NATURAL;

  BEGIN
    ----------------------------------------------------------------------------
    -- 1. Mise sous tension : reset de 1 ms, horloge figée, rien n'est émis.
    ----------------------------------------------------------------------------
    WAIT FOR DUREE_RESET;
    verifier(nb_recus = 0 AND nb_tx_dv = 0, "émission pendant le reset de mise sous tension");
    verifier(envoye = '0', "envoye à '1' pendant le reset");
    rst_b <= '1';
    t_release := NOW;
    verifier_message_apres_reset(t_release, "mise sous tension");
    verifier(nb_recus = MESSAGE_ATTENDU'LENGTH, "mise sous tension : trames en trop");
    -- Garde : 572 cycles (1,14 ms) puis 'R' ; message complet en moins de 18 ms.
    verifier(debuts_trames(0) - t_release < DUREE_TRAME + 200 us,
             "premier caractère trop tardif : " & TIME'IMAGE(debuts_trames(0) - t_release));
    verifier(NOW - t_release < 18 ms, "message trop lent : " & TIME'IMAGE(NOW - t_release));

    ----------------------------------------------------------------------------
    -- 2. Message envoyé une seule fois : rien de plus pendant 20 ms.
    ----------------------------------------------------------------------------
    nb_avant := nb_tx_dv;
    WAIT FOR 20 ms;
    verifier(nb_recus = MESSAGE_ATTENDU'LENGTH, "octets émis après la fin du message");
    verifier(nb_tx_dv = nb_avant, "demandes d'envoi après la fin du message");
    verifier(envoye = '1', "envoye retombé sans reset");

    ----------------------------------------------------------------------------
    -- 3. Reset dans le cycle qui suit une demande prise par UART_TX : UART_TX
    --    est figé dans l'état de début de trame avec o_TX_Active encore à '0'.
    --    Le message relancé doit malgré tout arriver complet.
    ----------------------------------------------------------------------------
    rst_b <= '0'; WAIT FOR DUREE_RESET; rst_b <= '1';
    -- nb_tx_dv s'incrémente au front où UART_TX lit tx_dv : on est donc juste
    -- après la prise de la 4e demande du message relancé.
    WAIT UNTIL nb_tx_dv = nb_avant + 4 FOR 10 ms;
    WAIT FOR 100 ns;
    verifier(tx_actif = '0', "scénario 3 : o_TX_Active devrait encore être à '0'");
    rst_b <= '0';
    WAIT FOR DUREE_RESET;
    rst_b <= '1';
    t_release := NOW;
    verifier_message_apres_reset(t_release, "reset juste après une demande");

    ----------------------------------------------------------------------------
    -- 4. Reset au milieu d'une trame : la ligne reste figée 1 ms (trame
    --    abîmée côté PC), puis le message complet doit suivre.
    ----------------------------------------------------------------------------
    rst_b <= '0'; WAIT FOR DUREE_RESET; rst_b <= '1';
    nb_avant := nb_recus;
    WAIT UNTIL nb_recus = nb_avant + 5 FOR 20 ms;
    WAIT UNTIL FALLING_EDGE(tx_ligne) FOR 2 ms;
    WAIT FOR 3 * DUREE_BIT + 10 us;
    rst_b <= '0';
    WAIT FOR DUREE_RESET;
    rst_b <= '1';
    t_release := NOW;
    verifier_message_apres_reset(t_release, "reset au milieu d'une trame");

    -- Bilan.
    IF nb_echecs = 0 AND erreurs_dv = 0 AND erreurs_prise = 0 THEN
      REPORT "TEST OK : tb_message_demarrage (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_message_demarrage (" & INTEGER'IMAGE(nb_echecs + erreurs_dv + erreurs_prise) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    std.env.finish;
  END PROCESS verification;

END ARCHITECTURE tb_message_demarrage_arch;
