--------------------------------------------------------------------------------
-- Fichier      : pwm_gen.vhd
-- Entité       : pwm_gen
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Générateur de signal PWM (modulation de largeur
--                d'impulsion). Il règle la vitesse des moteurs (entrées
--                ENABLE du pont en H L293D, PWM 5 kHz imposée par le cahier
--                des charges §6) et la luminosité de LED7 (niveau batterie).
-- Principe     : Un compteur tourne de 0 à 99 au rythme de l'horloge
--                système de 500 kHz : une période dure 100 x 2 µs = 200 µs,
--                soit 5 kHz. La sortie vaut '1' tant que le compteur est
--                inférieur à la consigne : la consigne est donc directement
--                le rapport cyclique en pourcent (0 = toujours '0',
--                100 = toujours '1').
-- Entrées      : clk   : horloge système 500 kHz.
--                rst_b : reset asynchrone actif à '0' (sortie à '0', moteur
--                        arrêté).
--                duty  : rapport cyclique voulu, en % (0 à 100).
-- Sorties      : pwm   : signal PWM, sortie d'une bascule.
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY pwm_gen IS
  PORT (
    clk   : IN  STD_LOGIC;              -- horloge système 500 kHz
    rst_b : IN  STD_LOGIC;              -- reset asynchrone, actif à '0'
    duty  : IN  UNSIGNED(6 DOWNTO 0);   -- rapport cyclique en % (0 à 100)
    pwm   : OUT STD_LOGIC               -- signal PWM
  );
END ENTITY pwm_gen;

ARCHITECTURE pwm_gen_arch OF pwm_gen IS

  -- Période PWM : 100 cycles de 2 µs = 200 µs (5 kHz).
  CONSTANT PERIODE : NATURAL := 100;

  -- Position dans la période en cours (0 à 99).
  SIGNAL compteur : NATURAL RANGE 0 TO PERIODE - 1;
  -- Bascule de sortie.
  SIGNAL pwm_reg  : STD_LOGIC;

BEGIN

  modulation : PROCESS (clk, rst_b)
  BEGIN
    IF rst_b = '0' THEN
      -- Reset : sortie à '0', donc moteur arrêté et LED éteinte.
      compteur <= 0;
      pwm_reg  <= '0';
    ELSIF RISING_EDGE(clk) THEN
      -- Avance dans la période, retour à 0 après 99.
      IF compteur = PERIODE - 1 THEN
        compteur <= 0;
      ELSE
        compteur <= compteur + 1;
      END IF;

      -- Sortie à '1' pendant les "duty" premiers cycles de la période.
      IF compteur < TO_INTEGER(duty) THEN
        pwm_reg <= '1';
      ELSE
        pwm_reg <= '0';
      END IF;
    END IF;
  END PROCESS modulation;

  pwm <= pwm_reg;

END ARCHITECTURE pwm_gen_arch;
