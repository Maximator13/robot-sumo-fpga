--------------------------------------------------------------------------------
-- Fichier      : circuit_reset.vhd
-- Entité       : circuit_reset
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Génère le reset général rst_b du robot (cahier des charges
--                §6.1) : rst_b, actif à '0', reste à '0' pendant 1 ms puis
--                repasse à '1', à la mise sous tension ET après chaque appui
--                sur le bouton poussoir BP1 de la carte d'interface.
-- Principe     : 1) Le bouton BP1 (actif à '0', déjà filtré par R2/C3 sur la
--                   carte) est asynchrone de l'horloge : il traverse deux
--                   bascules de synchronisation pour éviter la métastabilité.
--                2) Un compteur de durée de reset est remis à 0 tant que BP1
--                   est enfoncé, puis compte jusqu'à RESET_CYCLES_G
--                   (50 000 périodes de 20 ns = 1 ms). Tant que le compte
--                   n'est pas atteint, rst_b vaut '0'.
--                3) À la mise sous tension, le FPGA charge les valeurs
--                   initiales des registres : compteur à 0 et rst_b à '0', donc
--                   le reset de 1 ms démarre tout seul sans entrée de reset
--                   (ce module EST la source du reset, il n'en reçoit pas).
--                4) rst_b sort directement d'une bascule : aucun parasite
--                   combinatoire ne peut atteindre les resets asynchrones.
--                Conséquence : bouton maintenu = robot maintenu en reset ; le
--                robot redémarre 1 ms après le relâchement de BP1.
-- Entrées      : clk_in : horloge 50 MHz de la DE0-Nano.
--                bp_in  : bouton BP1, '0' quand il est enfoncé.
-- Sorties      : rst_b  : reset général actif à '0', vers rst_in_b du
--                         diviseur de fréquence et rst_b de tous les modules.
-- Génériques   : RESET_CYCLES_G : durée du reset en périodes de clk_in
--                (défaut RESET_CYCLES = 50 000, soit 1 ms à 50 MHz).
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY circuit_reset IS
  GENERIC (
    RESET_CYCLES_G : NATURAL := RESET_CYCLES   -- durée du reset (cycles de 50 MHz)
  );
  PORT (
    clk_in : IN  STD_LOGIC;   -- horloge 50 MHz
    bp_in  : IN  STD_LOGIC;   -- bouton BP1, actif à '0'
    rst_b  : OUT STD_LOGIC    -- reset général, actif à '0'
  );
END ENTITY circuit_reset;

ARCHITECTURE circuit_reset_arch OF circuit_reset IS

  -- Synchronisation du bouton sur l'horloge 50 MHz (2 bascules en série).
  -- Valeur de mise sous tension '1' : bouton considéré relâché au démarrage.
  SIGNAL bp_meta : STD_LOGIC := '1';   -- 1re bascule (peut être métastable)
  SIGNAL bp_sync : STD_LOGIC := '1';   -- 2e bascule (niveau stable, utilisable)

  -- Durée écoulée depuis le dernier appui (ou la mise sous tension).
  -- Valeur de mise sous tension 0 : le reset démarre seul.
  SIGNAL compteur  : NATURAL RANGE 0 TO RESET_CYCLES_G := 0;

  -- Bascule de sortie. Valeur de mise sous tension '0' : reset actif dès le
  -- chargement du FPGA.
  SIGNAL rst_b_reg : STD_LOGIC := '0';

BEGIN

  -- Synchronisation de BP1 et temporisation du reset.
  generation_reset : PROCESS (clk_in)
  BEGIN
    IF RISING_EDGE(clk_in) THEN
      -- Double bascule : le niveau du bouton n'est utilisé qu'après deux
      -- fronts d'horloge.
      bp_meta <= bp_in;
      bp_sync <= bp_meta;

      IF bp_sync = '0' THEN
        -- Bouton enfoncé : on maintient le reset et on recommence le
        -- décompte de 1 ms depuis le début.
        compteur  <= 0;
        rst_b_reg <= '0';
      ELSIF compteur < RESET_CYCLES_G THEN
        -- Durée de reset pas encore écoulée : reset toujours actif.
        compteur  <= compteur + 1;
        rst_b_reg <= '0';
      ELSE
        -- 1 ms écoulée sans appui : on libère le système.
        rst_b_reg <= '1';
      END IF;
    END IF;
  END PROCESS generation_reset;

  -- Sortie prise directement sur la bascule (pas de parasite sur le reset).
  rst_b <= rst_b_reg;

END ARCHITECTURE circuit_reset_arch;
