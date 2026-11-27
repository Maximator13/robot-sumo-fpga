--------------------------------------------------------------------------------
-- Fichier      : batterie_monitor.vhd
-- Entité       : batterie_monitor
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Surveille la tension des piles (cahier des charges §6.4) :
--                LED7 s'allume d'autant plus fort que la tension est haute
--                (10 % à 7,2 V, 100 % à 9 V) ; sous 7,2 V, LED7 clignote à
--                1 Hz et batt_vide demande l'arrêt des moteurs.
-- Principe     : Horloge système 500 kHz.
--                - Codes ADC de référence (schéma : diode D1 de 0,7 V puis
--                  pont 20 k / 10 k, ADC 12 bits sous 3,3 V) :
--                  code = (Vbat - 0,7) / 3 x 4096 / 3,3, soit 3434 à 9 V et
--                  2689 à 7,2 V.
--                - Luminosité : rapport cyclique = 10 + (code - 2689) x 90 / 745
--                  en %. La pente 90 / 745 est approchée par 31 / 256 : diviser
--                  par 256 revient à décaler de 8 bits, sans diviseur
--                  matériel. Le résultat est envoyé à un générateur PWM.
--                - Batterie vide : si le code reste sous 2689 pendant
--                  FILTRE_BATT_CYCLES_G cycles consécutifs, batt_vide passe à
--                  '1' jusqu'au reset. Le filtre ignore les chutes brèves
--                  (démarrage des moteurs).
--                - Clignotement : batterie vide, LED7 alterne
--                  CLIGNO_DEMI_CYCLES_G cycles allumée puis éteinte.
-- Entrées      : clk, rst_b    : horloge 500 kHz, reset asynchrone actif à '0'.
--                code_batterie : code ADC de la tension batterie.
-- Sorties      : led7      : témoin de batterie (PWM ou clignotement).
--                batt_vide : '1' quand la batterie est vide (moteurs arrêtés).
-- Génériques   : CLIGNO_DEMI_CYCLES_G : demi-période du clignotement
--                (250 000 cycles = 0,5 s, soit 1 Hz).
--                FILTRE_BATT_CYCLES_G : temps sous le seuil avant batterie
--                vide (500 000 cycles = 1 s).
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY batterie_monitor IS
  GENERIC (
    CLIGNO_DEMI_CYCLES_G : NATURAL := 250_000;   -- 0,5 s à 500 kHz
    FILTRE_BATT_CYCLES_G : NATURAL := 500_000    -- 1 s à 500 kHz
  );
  PORT (
    clk           : IN  STD_LOGIC;               -- horloge système 500 kHz
    rst_b         : IN  STD_LOGIC;               -- reset asynchrone, actif à '0'
    code_batterie : IN  UNSIGNED(11 DOWNTO 0);   -- tension batterie (code ADC)
    led7          : OUT STD_LOGIC;               -- témoin batterie
    batt_vide     : OUT STD_LOGIC                -- batterie vide
  );
END ENTITY batterie_monitor;

ARCHITECTURE batterie_monitor_arch OF batterie_monitor IS

  -- Codes ADC à 9 V (LED7 à 100 %) et à 7,2 V (10 %, seuil batterie vide).
  CONSTANT BAT_CODE_MAX : NATURAL := 3434;
  CONSTANT BAT_CODE_MIN : NATURAL := 2689;

  -- Rapport cyclique de LED7 calculé (combinatoire) puis registré.
  SIGNAL duty_calcule : NATURAL RANGE 0 TO 100;
  SIGNAL duty_led     : UNSIGNED(6 DOWNTO 0);
  SIGNAL pwm_led      : STD_LOGIC;

  -- Cycles consécutifs sous le seuil, et décision mémorisée.
  SIGNAL duree_sous_seuil : NATURAL RANGE 0 TO FILTRE_BATT_CYCLES_G;
  SIGNAL batt_vide_reg    : STD_LOGIC;

  -- Clignotement : position dans la demi-période et phase allumée / éteinte.
  SIGNAL compteur_cligno : NATURAL RANGE 0 TO CLIGNO_DEMI_CYCLES_G;
  SIGNAL phase_cligno    : STD_LOGIC;

  SIGNAL led7_reg : STD_LOGIC;

  -- Générateur PWM (même composant que pour les moteurs, cours VHDL §7.1).
  COMPONENT pwm_gen IS
    PORT (clk   : IN  STD_LOGIC;
          rst_b : IN  STD_LOGIC;
          duty  : IN  UNSIGNED(6 DOWNTO 0);
          pwm   : OUT STD_LOGIC);
  END COMPONENT pwm_gen;

BEGIN

  -- Loi de LED7 : 100 % au-dessus de 9 V, 10 % sous 7,2 V, droite entre les deux.
  duty_calcule <= 100 WHEN TO_INTEGER(code_batterie) >= BAT_CODE_MAX ELSE
                  10  WHEN TO_INTEGER(code_batterie) <  BAT_CODE_MIN ELSE
                  10 + ((TO_INTEGER(code_batterie) - BAT_CODE_MIN) * 31) / 256;

  surveillance : PROCESS (clk, rst_b)
  BEGIN
    IF rst_b = '0' THEN
      duty_led         <= (OTHERS => '0');
      duree_sous_seuil <= 0;
      batt_vide_reg    <= '0';
      compteur_cligno  <= 0;
      phase_cligno     <= '1';
      led7_reg         <= '0';
    ELSIF RISING_EDGE(clk) THEN

      -- Consigne de luminosité registrée vers le générateur PWM.
      duty_led <= TO_UNSIGNED(duty_calcule, 7);

      -- Filtre : batterie vide après FILTRE_BATT_CYCLES_G cycles consécutifs
      -- sous le seuil ; une seule mesure correcte relance le décompte.
      IF batt_vide_reg = '0' THEN
        IF TO_INTEGER(code_batterie) < BAT_CODE_MIN THEN
          IF duree_sous_seuil = FILTRE_BATT_CYCLES_G - 1 THEN
            batt_vide_reg <= '1';
          ELSE
            duree_sous_seuil <= duree_sous_seuil + 1;
          END IF;
        ELSE
          duree_sous_seuil <= 0;
        END IF;
      END IF;

      -- Clignotement 1 Hz, uniquement quand la batterie est vide.
      IF batt_vide_reg = '1' THEN
        IF compteur_cligno = CLIGNO_DEMI_CYCLES_G - 1 THEN
          compteur_cligno <= 0;
          phase_cligno    <= NOT phase_cligno;
        ELSE
          compteur_cligno <= compteur_cligno + 1;
        END IF;
      END IF;

      -- LED7 : clignotement si batterie vide, sinon luminosité PWM.
      IF batt_vide_reg = '1' THEN
        led7_reg <= phase_cligno;
      ELSE
        led7_reg <= pwm_led;
      END IF;

    END IF;
  END PROCESS surveillance;

  u_pwm_gen : pwm_gen
    PORT MAP (clk => clk, rst_b => rst_b, duty => duty_led, pwm => pwm_led);

  led7      <= led7_reg;
  batt_vide <= batt_vide_reg;

END ARCHITECTURE batterie_monitor_arch;
