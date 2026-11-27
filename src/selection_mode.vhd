--------------------------------------------------------------------------------
-- Fichier      : selection_mode.vhd
-- Entité       : selection_mode
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Choisit le mode de fonctionnement du robot à partir des
--                caractères envoyés par le PC (cahier des charges §1) :
--                'c' = combat SUMO, 's' = suiveur de ligne. Après un reset,
--                aucun mode n'est actif (robot en attente de commande).
-- Principe     : Deux bascules mémorisent le mode courant. À chaque octet
--                reçu (impulsion rx_dv de UART_RX), l'octet est comparé aux
--                codes ASCII de 'c' et de 's' ; tout autre caractère est
--                ignoré. Le mode peut être changé à tout moment.
-- Entrées      : clk      : horloge système 500 kHz.
--                rst_b    : reset asynchrone actif à '0' (retour en attente).
--                rx_dv    : '1' pendant un cycle quand un octet est reçu.
--                rx_octet : octet reçu du PC.
-- Sorties      : mode_combat  : '1' en mode combat SUMO.
--                mode_suiveur : '1' en mode suiveur de ligne.
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY selection_mode IS
  PORT (
    clk          : IN  STD_LOGIC;                      -- horloge système 500 kHz
    rst_b        : IN  STD_LOGIC;                      -- reset asynchrone, actif à '0'
    rx_dv        : IN  STD_LOGIC;                      -- un octet vient d'être reçu
    rx_octet     : IN  STD_LOGIC_VECTOR(7 DOWNTO 0);   -- octet reçu du PC
    mode_combat  : OUT STD_LOGIC;                      -- mode combat actif
    mode_suiveur : OUT STD_LOGIC                       -- mode suiveur actif
  );
END ENTITY selection_mode;

ARCHITECTURE selection_mode_arch OF selection_mode IS

  -- Codes ASCII des commandes.
  CONSTANT CAR_COMBAT  : STD_LOGIC_VECTOR(7 DOWNTO 0) := x"63";  -- 'c'
  CONSTANT CAR_SUIVEUR : STD_LOGIC_VECTOR(7 DOWNTO 0) := x"73";  -- 's'

  SIGNAL combat_reg  : STD_LOGIC;
  SIGNAL suiveur_reg : STD_LOGIC;

BEGIN

  memorisation_mode : PROCESS (clk, rst_b)
  BEGIN
    IF rst_b = '0' THEN
      -- Après la mise sous tension ou BP1 : en attente d'une commande.
      combat_reg  <= '0';
      suiveur_reg <= '0';
    ELSIF RISING_EDGE(clk) THEN
      IF rx_dv = '1' THEN
        -- Seules les commandes connues changent le mode.
        IF rx_octet = CAR_COMBAT THEN
          combat_reg  <= '1';
          suiveur_reg <= '0';
        ELSIF rx_octet = CAR_SUIVEUR THEN
          combat_reg  <= '0';
          suiveur_reg <= '1';
        END IF;
      END IF;
    END IF;
  END PROCESS memorisation_mode;

  mode_combat  <= combat_reg;
  mode_suiveur <= suiveur_reg;

END ARCHITECTURE selection_mode_arch;
