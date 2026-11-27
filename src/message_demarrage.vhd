--------------------------------------------------------------------------------
-- Fichier      : message_demarrage.vhd
-- Entité       : message_demarrage
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Envoie une fois au PC, après chaque reset, le message
--                "Robot PRET !" suivi de '\n' et '\r' (cahier des charges
--                §1), en pilotant l'émetteur série UART_TX.
-- Principe     : Machine à états de Moore (style du cours, chapitre 16) qui
--                envoie les 14 octets un par un :
--                - ATTENTE_UART_LIBRE : attend que l'émetteur soit au repos
--                  (tx_actif = '0'), puis passe à l'octet suivant ou termine ;
--                - EMISSION : tx_dv = '1' pendant un seul cycle, l'émetteur
--                  prend l'octet ;
--                - ATTENTE_DEBUT : attend le début de la trame (tx_actif =
--                  '1'), puis passe à l'octet suivant ;
--                - TERMINE : message envoyé, plus rien jusqu'au prochain reset.
--                Le message est rangé dans un tableau de constantes ; "rang"
--                désigne l'octet à envoyer.
-- Entrées      : clk      : horloge système 500 kHz.
--                rst_b    : reset asynchrone actif à '0' (message renvoyé).
--                tx_actif : o_TX_Active de UART_TX, '1' pendant une trame.
-- Sorties      : tx_dv    : demande d'envoi d'un octet (i_TX_DV de UART_TX).
--                tx_octet : octet à envoyer (i_TX_Byte de UART_TX).
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY message_demarrage IS
  PORT (
    clk      : IN  STD_LOGIC;                      -- horloge système 500 kHz
    rst_b    : IN  STD_LOGIC;                      -- reset asynchrone, actif à '0'
    tx_actif : IN  STD_LOGIC;                      -- émetteur en cours de trame
    tx_dv    : OUT STD_LOGIC;                      -- demande d'envoi d'un octet
    tx_octet : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)    -- octet à envoyer
  );
END ENTITY message_demarrage;

ARCHITECTURE message_demarrage_arch OF message_demarrage IS

  -- Message "Robot PRET !\n\r" en codes ASCII (14 octets).
  CONSTANT NB_OCTETS : NATURAL := 14;
  TYPE t_message IS ARRAY (0 TO NB_OCTETS - 1) OF STD_LOGIC_VECTOR(7 DOWNTO 0);
  CONSTANT MESSAGE : t_message := (
    x"52", x"6F", x"62", x"6F", x"74", x"20",   -- R o b o t (espace)
    x"50", x"52", x"45", x"54", x"20", x"21",   -- P R E T (espace) !
    x"0A",                                      -- '\n' : ligne suivante
    x"0D"                                       -- '\r' : retour en début de ligne
  );

  -- États de la machine d'envoi (codage laissé au synthétiseur).
  TYPE t_etat_message IS (ATTENTE_UART_LIBRE, EMISSION, ATTENTE_DEBUT, TERMINE);
  SIGNAL etat : t_etat_message;

  -- Rang de l'octet à envoyer (NB_OCTETS = tout est parti).
  SIGNAL rang : NATURAL RANGE 0 TO NB_OCTETS;

BEGIN

  sequencement : PROCESS (clk, rst_b)
  BEGIN
    IF rst_b = '0' THEN
      -- Reset : le message repart du premier caractère.
      etat <= ATTENTE_UART_LIBRE;
      rang <= 0;
    ELSIF RISING_EDGE(clk) THEN
      CASE etat IS

        WHEN ATTENTE_UART_LIBRE =>
          -- Émetteur au repos : octet suivant, ou fin du message.
          IF tx_actif = '0' THEN
            IF rang = NB_OCTETS THEN
              etat <= TERMINE;
            ELSE
              etat <= EMISSION;
            END IF;
          END IF;

        WHEN EMISSION =>
          -- tx_dv vaut '1' pendant ce seul cycle.
          etat <= ATTENTE_DEBUT;

        WHEN ATTENTE_DEBUT =>
          -- La trame a démarré : l'octet est pris en charge.
          IF tx_actif = '1' THEN
            rang <= rang + 1;
            etat <= ATTENTE_UART_LIBRE;
          END IF;

        WHEN TERMINE =>
          -- Message envoyé : on reste ici jusqu'au prochain reset.
          etat <= TERMINE;

      END CASE;
    END IF;
  END PROCESS sequencement;

  -- Sorties de Moore, décodées depuis l'état et le rang.
  tx_dv    <= '1' WHEN etat = EMISSION ELSE '0';
  tx_octet <= MESSAGE(rang) WHEN rang < NB_OCTETS ELSE x"00";

END ARCHITECTURE message_demarrage_arch;
