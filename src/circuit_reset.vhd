--------------------------------------------------------------------------------
-- Fichier      : circuit_reset.vhd
-- Entité       : circuit_reset
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Génère le reset général rst_b du robot (cahier des charges
--                §6.1) : rst_b, actif à '0', reste à '0' pendant 1 ms puis
--                repasse à '1', à la mise sous tension et après chaque appui
--                sur le bouton BP1 de la carte d'interface.
-- Principe     : - BP1 est asynchrone de l'horloge : il passe par deux
--                  bascules en série avant d'être utilisé (la première peut
--                  être métastable, la seconde donne un niveau stable).
--                - Un compteur est remis à 0 tant que BP1 est enfoncé, puis
--                  compte jusqu'à RESET_CYCLES_G (50 000 x 20 ns = 1 ms).
--                  Tant que le compte n'est pas atteint, rst_b vaut '0'.
--                - Ce module est la source du reset : il n'a pas d'entrée de
--                  reset. À la mise sous tension, le FPGA charge les valeurs
--                  initiales des registres (compteur à 0, rst_b à '0'), ce
--                  qui lance le reset de 1 ms tout seul.
--                - rst_b sort directement d'une bascule (pas de parasite).
-- Entrées      : clk_in : horloge 50 MHz de la DE0-Nano.
--                bp_in  : bouton BP1, '0' quand il est enfoncé.
-- Sorties      : rst_b  : reset général actif à '0'.
-- Génériques   : RESET_CYCLES_G : durée du reset en périodes de clk_in
--                (50 000, soit 1 ms à 50 MHz).
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY circuit_reset IS
  GENERIC (
    RESET_CYCLES_G : NATURAL := 50_000   -- 1 ms à 50 MHz
  );
  PORT (
    clk_in : IN  STD_LOGIC;   -- horloge 50 MHz
    bp_in  : IN  STD_LOGIC;   -- bouton BP1, actif à '0'
    rst_b  : OUT STD_LOGIC    -- reset général, actif à '0'
  );
END ENTITY circuit_reset;

ARCHITECTURE circuit_reset_arch OF circuit_reset IS

  -- Synchronisation du bouton (2 bascules). Valeur de mise sous tension '1' :
  -- bouton considéré relâché au démarrage.
  SIGNAL bp_meta : STD_LOGIC := '1';   -- 1re bascule (peut être métastable)
  SIGNAL bp_sync : STD_LOGIC := '1';   -- 2e bascule (niveau stable)

  -- Temps écoulé depuis le dernier appui ou la mise sous tension.
  SIGNAL compteur  : NATURAL RANGE 0 TO RESET_CYCLES_G := 0;

  -- Bascule de sortie : '0' dès la mise sous tension (reset actif).
  SIGNAL rst_b_reg : STD_LOGIC := '0';

BEGIN

  generation_reset : PROCESS (clk_in)
  BEGIN
    IF RISING_EDGE(clk_in) THEN
      -- Double bascule de synchronisation du bouton.
      bp_meta <= bp_in;
      bp_sync <= bp_meta;

      IF bp_sync = '0' THEN
        -- Bouton enfoncé : reset maintenu, décompte de 1 ms relancé.
        compteur  <= 0;
        rst_b_reg <= '0';
      ELSIF compteur < RESET_CYCLES_G THEN
        -- 1 ms pas encore écoulée : reset toujours actif.
        compteur  <= compteur + 1;
        rst_b_reg <= '0';
      ELSE
        -- 1 ms écoulée sans appui : le système démarre.
        rst_b_reg <= '1';
      END IF;
    END IF;
  END PROCESS generation_reset;

  rst_b <= rst_b_reg;

END ARCHITECTURE circuit_reset_arch;
