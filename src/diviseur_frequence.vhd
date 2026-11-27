--------------------------------------------------------------------------------
-- Fichier      : diviseur_frequence.vhd
-- Entité       : diviseur_frequence
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Produit l'horloge système de 500 kHz qui cadence le robot,
--                à partir de l'oscillateur 50 MHz de la carte (cahier des
--                charges §6.2). Travailler à 500 kHz plutôt qu'à 50 MHz
--                réduit la consommation et suffit au robot.
-- Principe     : Un compteur de 0 à 49 avance à chaque front montant de
--                clk_in. Quand il atteint 49, il repart à 0 et la sortie
--                change d'état : 50 périodes à '1' puis 50 à '0', soit une
--                période de 100 x 20 ns = 2 µs (500 kHz), rapport cyclique
--                50 %. La sortie est celle d'une bascule (horloge propre),
--                déclarée comme horloge générée dans robot_sumo.sdc.
-- Entrées      : clk_in   : horloge 50 MHz (PIN_R8).
--                rst_in_b : reset asynchrone actif à '0' (rst_b du circuit de
--                           reset).
-- Sorties      : clk_500KHz : horloge système 500 kHz, à '0' pendant le reset.
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY diviseur_frequence IS
  PORT (
    clk_in     : IN  STD_LOGIC;   -- horloge 50 MHz
    rst_in_b   : IN  STD_LOGIC;   -- reset asynchrone, actif à '0'
    clk_500KHz : OUT STD_LOGIC    -- horloge système 500 kHz
  );
END ENTITY diviseur_frequence;

ARCHITECTURE diviseur_frequence_arch OF diviseur_frequence IS

  -- Demi-période de la sortie en périodes de 50 MHz : (50 MHz / 500 kHz) / 2.
  CONSTANT DEMI_PERIODE : NATURAL := 50;

  -- Périodes de clk_in écoulées depuis le dernier changement de la sortie.
  SIGNAL compteur       : NATURAL RANGE 0 TO DEMI_PERIODE - 1;
  -- Bascule qui porte l'horloge 500 kHz (nom repris dans robot_sumo.sdc).
  SIGNAL clk_500KHz_reg : STD_LOGIC;

BEGIN

  division : PROCESS (clk_in, rst_in_b)
  BEGIN
    IF rst_in_b = '0' THEN
      -- Pendant le reset, l'horloge système est figée à '0'.
      compteur       <= 0;
      clk_500KHz_reg <= '0';
    ELSIF RISING_EDGE(clk_in) THEN
      IF compteur = DEMI_PERIODE - 1 THEN
        -- Demi-période écoulée : la sortie change d'état.
        compteur       <= 0;
        clk_500KHz_reg <= NOT clk_500KHz_reg;
      ELSE
        compteur <= compteur + 1;
      END IF;
    END IF;
  END PROCESS division;

  clk_500KHz <= clk_500KHz_reg;

END ARCHITECTURE diviseur_frequence_arch;
