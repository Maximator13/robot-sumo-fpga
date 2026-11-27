--------------------------------------------------------------------------------
-- Fichier      : message_demarrage.vhd
-- Entité       : message_demarrage
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Envoie au PC, une seule fois après chaque reset (mise sous
--                tension ou appui sur BP1), le message imposé par le cahier
--                des charges §1 : "Robot PRET !" suivi de '\n' (LF, 0x0A,
--                passage à la ligne) puis '\r' (CR, 0x0D, retour chariot),
--                soit 14 octets. Il pilote l'émetteur UART_TX fourni.
-- Principe     : Machine à états de Moore (style du cours, chapitre 16) :
--                - GARDE : après un reset, attend que l'émetteur soit resté
--                  au repos (o_TX_Active = '0') pendant GARDE_CYCLES_G cycles
--                  consécutifs, soit un peu plus d'une trame (1,1 ms).
--                  Raison : UART_TX n'a pas de reset et l'horloge 500 kHz est
--                  arrêtée pendant le reset ; l'émetteur peut donc se
--                  retrouver figé au milieu d'une trame, ou juste après avoir
--                  accepté une demande (o_TX_Active encore à '0' pendant un
--                  cycle). La garde laisse finir cette trame, puis laisse la
--                  ligne au repos assez longtemps pour que le PC se
--                  resynchronise avant le premier caractère du message.
--                - ATTENTE_UART_LIBRE : attend que l'émetteur soit au repos.
--                  Si les 14 octets sont partis, le message est terminé ;
--                  sinon on émet l'octet suivant.
--                - EMISSION : impulsion d'un cycle sur tx_dv, l'octet courant
--                  étant présenté sur tx_octet.
--                - ATTENTE_DEBUT : attend que l'émetteur signale le début de
--                  la trame (o_TX_Active = '1', un cycle après la prise en
--                  compte de tx_dv), puis passe à l'octet suivant.
--                - TERMINE : plus rien n'est envoyé jusqu'au prochain reset.
--                Hors reset, une demande n'est faite que si l'émetteur est
--                au repos (UART_TX n'écoute tx_dv qu'au repos) : aucune
--                demande n'est perdue. Le texte du message est une table de
--                constantes (ROM).
-- Entrées      : clk      : horloge système 500 kHz.
--                rst_b    : reset asynchrone actif à '0' (relance le message).
--                tx_actif : o_TX_Active de UART_TX.
--                tx_fini  : o_TX_Done de UART_TX (non nécessaire au
--                           séquencement, conservé pour l'interface prévue).
-- Sorties      : tx_dv    : demande d'envoi vers i_TX_DV (1 cycle).
--                tx_octet : octet à envoyer, vers i_TX_Byte.
--                envoye   : '1' quand le message complet est parti.
-- Génériques   : GARDE_CYCLES_G : durée de repos exigée de l'émetteur après
--                un reset, en cycles de clk (défaut 11 bits de 52 cycles =
--                572 cycles = 1,14 ms à 500 kHz).
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY message_demarrage IS
  GENERIC (
    GARDE_CYCLES_G : NATURAL := 11 * UART_CLKS_PER_BIT   -- repos exigé après reset (cycles)
  );
  PORT (
    clk      : IN  STD_LOGIC;                      -- horloge système 500 kHz
    rst_b    : IN  STD_LOGIC;                      -- reset asynchrone, actif à '0'
    tx_actif : IN  STD_LOGIC;                      -- émetteur en cours de trame
    tx_fini  : IN  STD_LOGIC;                      -- fin de trame de l'émetteur (non utilisé)
    tx_dv    : OUT STD_LOGIC;                      -- demande d'envoi d'un octet
    tx_octet : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);   -- octet à envoyer
    envoye   : OUT STD_LOGIC                       -- message entièrement envoyé
  );
END ENTITY message_demarrage;

ARCHITECTURE message_demarrage_arch OF message_demarrage IS

  -- Code ASCII d'un caractère, sur 8 bits.
  FUNCTION ascii (c : CHARACTER) RETURN STD_LOGIC_VECTOR IS
  BEGIN
    RETURN STD_LOGIC_VECTOR(TO_UNSIGNED(CHARACTER'POS(c), 8));
  END FUNCTION ascii;

  -- Texte du message (CDC §1) : "Robot PRET !" puis LF puis CR.
  CONSTANT NB_OCTETS : NATURAL := 14;
  TYPE t_message IS ARRAY (0 TO NB_OCTETS - 1) OF STD_LOGIC_VECTOR(7 DOWNTO 0);
  CONSTANT MESSAGE : t_message := (
    ascii('R'), ascii('o'), ascii('b'), ascii('o'), ascii('t'), ascii(' '),
    ascii('P'), ascii('R'), ascii('E'), ascii('T'), ascii(' '), ascii('!'),
    x"0A",    -- '\n' : passage à la ligne suivante
    x"0D"     -- '\r' : retour en début de ligne
  );

  -- États de la machine d'envoi (codage laissé au synthétiseur).
  TYPE t_etat_message IS (GARDE, ATTENTE_UART_LIBRE, EMISSION, ATTENTE_DEBUT, TERMINE);
  SIGNAL etat : t_etat_message;

  -- Rang de l'octet à envoyer (NB_OCTETS = tout est parti).
  SIGNAL rang : NATURAL RANGE 0 TO NB_OCTETS;

  -- Nombre de cycles consécutifs d'émetteur au repos, compté dans GARDE.
  SIGNAL duree_repos : NATURAL RANGE 0 TO GARDE_CYCLES_G;

BEGIN

  -- Calcul de l'état futur, du rang de l'octet et de la garde (processus
  -- synchrone unique).
  sequencement : PROCESS (clk, rst_b)
  BEGIN
    IF rst_b = '0' THEN
      -- Reset : garde puis message depuis le premier caractère.
      etat        <= GARDE;
      rang        <= 0;
      duree_repos <= 0;
    ELSIF RISING_EDGE(clk) THEN
      CASE etat IS

        WHEN GARDE =>
          -- On exige un repos ininterrompu de l'émetteur : toute activité
          -- (fin d'une trame figée par le reset) relance le décompte.
          IF tx_actif /= '0' THEN
            duree_repos <= 0;
          ELSIF duree_repos < GARDE_CYCLES_G THEN
            duree_repos <= duree_repos + 1;
          ELSE
            etat <= ATTENTE_UART_LIBRE;
          END IF;

        WHEN ATTENTE_UART_LIBRE =>
          -- L'émetteur est au repos : message fini, ou octet suivant.
          IF tx_actif = '0' THEN
            IF rang = NB_OCTETS THEN
              etat <= TERMINE;
            ELSE
              etat <= EMISSION;
            END IF;
          END IF;

        WHEN EMISSION =>
          -- tx_dv est à '1' pendant ce seul cycle : UART_TX prend l'octet.
          etat <= ATTENTE_DEBUT;

        WHEN ATTENTE_DEBUT =>
          -- La trame a démarré : l'octet est pris en charge, on passe au
          -- suivant et on attend la fin de la trame.
          IF tx_actif = '1' THEN
            rang <= rang + 1;
            etat <= ATTENTE_UART_LIBRE;
          END IF;

        WHEN TERMINE =>
          -- Message envoyé : on n'envoie plus rien jusqu'au prochain reset.
          etat <= TERMINE;

        WHEN OTHERS =>
          etat <= GARDE;

      END CASE;
    END IF;
  END PROCESS sequencement;

  -- Sorties de Moore, décodées depuis l'état et le rang.
  tx_dv    <= '1' WHEN etat = EMISSION ELSE '0';
  tx_octet <= MESSAGE(rang) WHEN rang < NB_OCTETS ELSE (OTHERS => '0');
  envoye   <= '1' WHEN etat = TERMINE ELSE '0';

END ARCHITECTURE message_demarrage_arch;
