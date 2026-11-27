--------------------------------------------------------------------------------
-- Fichier      : selection_mode.vhd
-- Entité       : selection_mode
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Choisit le mode de fonctionnement du robot à partir des
--                caractères envoyés par le PC sur la liaison série
--                (cahier des charges §1) : 'c' passe en combat SUMO, 's'
--                passe en suiveur de ligne. Après la mise sous tension ou un
--                reset, le robot est en attente de commande.
-- Principe     : Un registre mémorise le mode courant. À chaque octet reçu
--                (impulsion rx_dv de UART_RX), l'octet est comparé aux codes
--                ASCII de 'c' et de 's' : le mode change seulement si l'un
--                des deux est reconnu ; tout autre caractère est ignoré et le
--                mode reste inchangé. Le mode peut être changé à tout moment.
-- Entrées      : clk      : horloge système 500 kHz.
--                rst_b    : reset asynchrone actif à '0' (retour en attente).
--                rx_dv    : o_RX_DV de UART_RX, '1' un cycle par octet reçu.
--                rx_octet : o_RX_Byte de UART_RX, octet reçu.
-- Sorties      : mode     : MODE_ATTENTE, MODE_COMBAT ou MODE_SUIVEUR.
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY selection_mode IS
  PORT (
    clk      : IN  STD_LOGIC;                      -- horloge système 500 kHz
    rst_b    : IN  STD_LOGIC;                      -- reset asynchrone, actif à '0'
    rx_dv    : IN  STD_LOGIC;                      -- un octet vient d'être reçu
    rx_octet : IN  STD_LOGIC_VECTOR(7 DOWNTO 0);   -- octet reçu du PC
    mode     : OUT t_mode                          -- mode de fonctionnement courant
  );
END ENTITY selection_mode;

ARCHITECTURE selection_mode_arch OF selection_mode IS

  -- Registre du mode courant.
  SIGNAL mode_reg : t_mode;

BEGIN

  memorisation_mode : PROCESS (clk, rst_b)
  BEGIN
    IF rst_b = '0' THEN
      -- Après la mise sous tension ou BP1 : en attente d'une commande.
      mode_reg <= MODE_ATTENTE;
    ELSIF RISING_EDGE(clk) THEN
      IF rx_dv = '1' THEN
        -- Un octet vient d'arriver : seules les commandes connues changent
        -- le mode, les autres caractères sont sans effet.
        IF rx_octet = CAR_COMBAT THEN
          mode_reg <= MODE_COMBAT;
        ELSIF rx_octet = CAR_SUIVEUR THEN
          mode_reg <= MODE_SUIVEUR;
        END IF;
      END IF;
    END IF;
  END PROCESS memorisation_mode;

  mode <= mode_reg;

END ARCHITECTURE selection_mode_arch;
