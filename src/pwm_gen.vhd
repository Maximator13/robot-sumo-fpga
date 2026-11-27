--------------------------------------------------------------------------------
-- Fichier      : pwm_gen.vhd
-- Entité       : pwm_gen
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Générateur de signal PWM (modulation de largeur
--                d'impulsion). Il pilote la vitesse des moteurs via les
--                entrées ENABLE du pont en H L293D (PWM 5 kHz imposée par le
--                cahier des charges §6) et la luminosité de LED7 (niveau de
--                batterie, §6.4).
-- Principe     : Un compteur tourne de 0 à PERIODE_CYCLES - 1 au rythme de
--                l'horloge système. La sortie vaut '1' tant que le compteur
--                est inférieur à la consigne, '0' ensuite. Avec l'horloge de
--                500 kHz et PERIODE_CYCLES = 100, la période est de 200 µs
--                (5 kHz) et la consigne s'exprime directement en pourcent.
--                La consigne n'est prise en compte qu'au début d'une période :
--                un changement de vitesse en cours de période ne produit donc
--                jamais d'impulsion tronquée. Une consigne supérieure à la
--                période est ramenée à la période (sortie toujours à '1').
--                La sortie est registrée (pas de parasite vers le pont en H).
-- Entrées      : clk   : horloge système 500 kHz.
--                rst_b : reset asynchrone actif à '0' (sortie forcée à '0',
--                        donc moteur arrêté).
--                duty  : consigne = nombre de cycles à '1' par période,
--                        soit le rapport cyclique en % quand PERIODE_CYCLES
--                        = 100 (0 = toujours '0', 100 = toujours '1').
-- Sorties      : pwm   : signal PWM.
-- Génériques   : PERIODE_CYCLES : période en cycles d'horloge (défaut
--                PWM_PERIODE_CYCLES = 100, soit 5 kHz à 500 kHz).
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY pwm_gen IS
  GENERIC (
    PERIODE_CYCLES : NATURAL := PWM_PERIODE_CYCLES   -- période PWM (cycles d'horloge)
  );
  PORT (
    clk   : IN  STD_LOGIC;   -- horloge système 500 kHz
    rst_b : IN  STD_LOGIC;   -- reset asynchrone, actif à '0'
    duty  : IN  t_duty;      -- consigne (cycles à '1' par période, en % si période = 100)
    pwm   : OUT STD_LOGIC    -- signal PWM
  );
END ENTITY pwm_gen;

ARCHITECTURE pwm_gen_arch OF pwm_gen IS

  -- Position dans la période en cours.
  SIGNAL compteur       : NATURAL RANGE 0 TO PERIODE_CYCLES - 1;
  -- Consigne figée pour la période en cours (déjà saturée à PERIODE_CYCLES).
  SIGNAL consigne_figee : NATURAL RANGE 0 TO PERIODE_CYCLES;
  -- Bascule de sortie.
  SIGNAL pwm_reg        : STD_LOGIC;

BEGIN

  modulation : PROCESS (clk, rst_b)
  BEGIN
    IF rst_b = '0' THEN
      -- Reset : sortie à '0' (moteur arrêté, LED éteinte) et consigne nulle.
      compteur       <= 0;
      consigne_figee <= 0;
      pwm_reg        <= '0';
    ELSIF RISING_EDGE(clk) THEN
      -- Avance dans la période, et en fin de période lecture de la nouvelle
      -- consigne, ramenée à la période si elle la dépasse.
      IF compteur = PERIODE_CYCLES - 1 THEN
        compteur <= 0;
        IF TO_INTEGER(duty) > PERIODE_CYCLES THEN
          consigne_figee <= PERIODE_CYCLES;
        ELSE
          consigne_figee <= TO_INTEGER(duty);
        END IF;
      ELSE
        compteur <= compteur + 1;
      END IF;

      -- Sortie à '1' pendant les consigne_figee premiers cycles de la période.
      IF compteur < consigne_figee THEN
        pwm_reg <= '1';
      ELSE
        pwm_reg <= '0';
      END IF;
    END IF;
  END PROCESS modulation;

  pwm <= pwm_reg;

END ARCHITECTURE pwm_gen_arch;
