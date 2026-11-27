--------------------------------------------------------------------------------
-- Fichier      : diviseur_frequence.vhd
-- Entité       : diviseur_frequence
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Produit l'horloge système de 500 kHz qui cadence tout le
--                robot, à partir de l'oscillateur 50 MHz de la carte
--                (cahier des charges §6.2). Travailler à 500 kHz plutôt qu'à
--                50 MHz réduit la consommation, et suffit largement aux
--                constantes de temps du robot (moteurs, capteurs, série).
-- Principe     : Un compteur de 0 à 49 avance à chaque front montant de
--                clk_in. À chaque fois qu'il atteint 49, il repart à 0 et la
--                bascule de sortie change d'état. La sortie reste donc 50
--                périodes à '1' puis 50 périodes à '0' : période de 100 x 20 ns
--                = 2 µs (500 kHz), rapport cyclique 50 %.
--                La sortie est directement la sortie d'une bascule (pas de
--                logique combinatoire), ce qui évite tout parasite sur une
--                horloge. Elle est déclarée dans robot_sumo.sdc comme horloge
--                générée (divide_by 100) pour l'analyse temporelle.
-- Entrées      : clk_in   : horloge 50 MHz de la DE0-Nano (PIN_R8).
--                rst_in_b : reset asynchrone actif bas, relié à rst_b du
--                           circuit de reset.
-- Sorties      : clk_500KHz : horloge système 500 kHz, à '0' pendant le reset.
-- Génériques   : aucun (rapport de division tiré de sumo_pkg : 50 MHz / 500 kHz)
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY diviseur_frequence IS
  PORT (
    clk_in     : IN  STD_LOGIC;   -- horloge 50 MHz
    rst_in_b   : IN  STD_LOGIC;   -- reset asynchrone, actif à '0'
    clk_500KHz : OUT STD_LOGIC    -- horloge système 500 kHz
  );
END ENTITY diviseur_frequence;

ARCHITECTURE diviseur_frequence_arch OF diviseur_frequence IS

  -- Nombre de périodes de clk_in pendant lesquelles la sortie garde le même
  -- niveau : la moitié du rapport de division, soit (50 MHz / 500 kHz) / 2 = 50.
  CONSTANT DEMI_PERIODE : NATURAL := (CLK_IN_HZ / CLK_SYS_HZ) / 2;

  -- Compte les périodes de clk_in écoulées depuis le dernier changement de
  -- niveau de la sortie (0 à DEMI_PERIODE - 1).
  SIGNAL compteur       : NATURAL RANGE 0 TO DEMI_PERIODE - 1;
  -- Bascule qui porte l'horloge 500 kHz (nom repris dans robot_sumo.sdc).
  SIGNAL clk_500KHz_reg : STD_LOGIC;

BEGIN

  -- Comptage et basculement de l'horloge de sortie.
  division : PROCESS (clk_in, rst_in_b)
  BEGIN
    IF rst_in_b = '0' THEN
      -- Pendant le reset, l'horloge système est figée à '0' : aucun module
      -- cadencé à 500 kHz ne voit de front montant.
      compteur       <= 0;
      clk_500KHz_reg <= '0';
    ELSIF RISING_EDGE(clk_in) THEN
      IF compteur = DEMI_PERIODE - 1 THEN
        -- Une demi-période de 500 kHz est écoulée : on inverse la sortie et
        -- on repart pour une nouvelle demi-période.
        compteur       <= 0;
        clk_500KHz_reg <= NOT clk_500KHz_reg;
      ELSE
        compteur <= compteur + 1;
      END IF;
    END IF;
  END PROCESS division;

  -- Sortie prise directement sur la bascule (horloge sans parasite).
  clk_500KHz <= clk_500KHz_reg;

END ARCHITECTURE diviseur_frequence_arch;
