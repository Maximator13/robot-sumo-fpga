--------------------------------------------------------------------------------
-- Fichier      : batterie_monitor.vhd
-- Entité       : batterie_monitor
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Surveille la tension des piles (cahier des charges §6.4) et
--                informe l'utilisateur par LED7 : luminosité proportionnelle
--                à la tension entre 7,2 V (10 %) et 9 V (100 %), puis
--                clignotement à 1 Hz quand la batterie est déclarée vide.
--                Fournit batt_vide, qui fait arrêter les moteurs.
-- Principe     : Horloge système 500 kHz, trois parties :
--                - Loi LED7 : rapport cyclique 10 + ((code - BAT_CODE_MIN) x
--                  PENTE_LED7_NUM) / 2**PENTE_LED7_DECALAGE en %, sans
--                  diviseur matériel (multiplication par une constante puis
--                  décalage). Saturé à 100 au-delà de 9 V et maintenu à 10
--                  sous 7,2 V tant que la batterie n'est pas déclarée vide.
--                  La consigne est registrée puis convertie en PWM par une
--                  instance de pwm_gen.
--                - Filtre batterie vide : un compteur compte les cycles
--                  CONSÉCUTIFS où le code est sous BAT_CODE_MIN ; un seul
--                  code >= BAT_CODE_MIN le remet à 0. Au bout de
--                  FILTRE_BATT_CYCLES_G cycles, batt_vide passe à '1' et y
--                  reste jusqu'au reset (piles épuisées : on ne repart pas
--                  sur une remontée de tension à vide). Le filtre ignore les
--                  chutes brèves (démarrage des moteurs) et le code nul
--                  présent juste après un reset, avant la première mesure
--                  de l'ADC.
--                - Clignotement : batterie vide, LED7 alterne
--                  CLIGNO_DEMI_CYCLES_G cycles allumée puis autant éteinte,
--                  en commençant par la phase allumée.
--                Les sorties led7 et batt_vide sont des bascules (pas de
--                parasite vers la LED ni vers la commande des moteurs).
-- Entrées      : clk           : horloge système 500 kHz.
--                rst_b         : reset asynchrone actif à '0'.
--                code_batterie : code ADC 12 bits de la voie BAT (registré
--                                par detection_capteurs).
-- Sorties      : led7          : PWM proportionnelle à la tension, ou
--                                clignotement 1 Hz si batterie vide.
--                batt_vide     : '1' (définitif jusqu'au reset) si la
--                                batterie est restée sous 7,2 V pendant
--                                FILTRE_BATT_CYCLES_G cycles consécutifs.
-- Génériques   : CLIGNO_DEMI_CYCLES_G : demi-période du clignotement en
--                                cycles (défaut 250 000 = 0,5 s, soit 1 Hz).
--                FILTRE_BATT_CYCLES_G : durée sous le seuil avant batterie
--                                vide, en cycles, >= 1 (défaut 500 000 = 1 s).
--                PERIODE_PWM_G        : période de la PWM de LED7 en cycles
--                                (défaut 100 = 5 kHz ; la consigne est en %
--                                pour une période de 100).
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY batterie_monitor IS
  GENERIC (
    CLIGNO_DEMI_CYCLES_G : NATURAL := CLIGNO_DEMI_CYCLES;   -- demi-période clignotement (cycles)
    FILTRE_BATT_CYCLES_G : NATURAL := FILTRE_BATT_CYCLES;   -- durée sous le seuil (cycles)
    PERIODE_PWM_G        : NATURAL := PWM_PERIODE_CYCLES    -- période PWM LED7 (cycles)
  );
  PORT (
    clk           : IN  STD_LOGIC;   -- horloge système 500 kHz
    rst_b         : IN  STD_LOGIC;   -- reset asynchrone, actif à '0'
    code_batterie : IN  t_mot_adc;   -- tension piles mesurée (code ADC)
    led7          : OUT STD_LOGIC;   -- témoin batterie
    batt_vide     : OUT STD_LOGIC    -- batterie déclarée vide
  );
END ENTITY batterie_monitor;

ARCHITECTURE batterie_monitor_arch OF batterie_monitor IS

  -- Rapport cyclique minimal (7,2 V) et maximal (9 V) de LED7, en %.
  CONSTANT DUTY_MIN : NATURAL := 10;
  CONSTANT DUTY_MAX : NATURAL := 100;

  -- Rapport cyclique calculé à partir du code (combinatoire) et sa copie
  -- registrée envoyée au générateur PWM.
  SIGNAL duty_calcule : NATURAL RANGE 0 TO DUTY_MAX;
  SIGNAL duty_led     : t_duty;
  SIGNAL pwm_led      : STD_LOGIC;

  -- Filtre : cycles consécutifs sous le seuil, et décision mémorisée.
  SIGNAL duree_sous_seuil : NATURAL RANGE 0 TO FILTRE_BATT_CYCLES_G - 1;
  SIGNAL batt_vide_reg    : STD_LOGIC;

  -- Clignotement : position dans la demi-période et phase (allumée/éteinte).
  SIGNAL compteur_cligno : NATURAL RANGE 0 TO CLIGNO_DEMI_CYCLES_G - 1;
  SIGNAL phase_cligno    : STD_LOGIC;

  -- Bascule de sortie de LED7.
  SIGNAL led7_reg : STD_LOGIC;

  -- Générateur PWM (même composant que pour les moteurs, cours VHDL §7.1) :
  -- transforme la consigne en % en signal de luminosité pour LED7.
  COMPONENT pwm_gen IS
    GENERIC (PERIODE_CYCLES : NATURAL);
    PORT (clk   : IN  STD_LOGIC;
          rst_b : IN  STD_LOGIC;
          duty  : IN  t_duty;
          pwm   : OUT STD_LOGIC);
  END COMPONENT pwm_gen;

BEGIN

  ------------------------------------------------------------------------------
  -- Loi LED7 (CDC §6.4) : droite de 10 % à 7,2 V jusqu'à 100 % à 9 V,
  -- bornée aux deux extrémités. Sous le seuil (batterie en cours de
  -- filtrage, ou code nul après un reset), on garde le minimum de 10 %.
  ------------------------------------------------------------------------------
  duty_calcule <= DUTY_MAX WHEN TO_INTEGER(code_batterie) >= BAT_CODE_MAX ELSE
                  DUTY_MIN WHEN TO_INTEGER(code_batterie) <  BAT_CODE_MIN ELSE
                  DUTY_MIN + ((TO_INTEGER(code_batterie) - BAT_CODE_MIN) * PENTE_LED7_NUM)
                             / (2 ** PENTE_LED7_DECALAGE);

  ------------------------------------------------------------------------------
  -- Partie séquentielle : consigne LED7, filtre batterie vide, clignotement
  -- et sortie LED7.
  ------------------------------------------------------------------------------
  surveillance : PROCESS (clk, rst_b)
  BEGIN
    IF rst_b = '0' THEN
      -- Reset : batterie supposée bonne, LED éteinte, clignotement prêt à
      -- démarrer par sa phase allumée.
      duty_led         <= (OTHERS => '0');
      duree_sous_seuil <= 0;
      batt_vide_reg    <= '0';
      compteur_cligno  <= 0;
      phase_cligno     <= '1';
      led7_reg         <= '0';
    ELSIF RISING_EDGE(clk) THEN

      -- Consigne de luminosité registrée (coupe le chemin combinatoire du
      -- calcul avant le générateur PWM).
      duty_led <= TO_UNSIGNED(duty_calcule, t_duty'LENGTH);

      -- Filtre : la batterie n'est déclarée vide qu'après
      -- FILTRE_BATT_CYCLES_G cycles consécutifs sous le seuil ; une seule
      -- mesure correcte relance le décompte. Une fois vide, on ne revient
      -- plus en arrière avant le reset.
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

      -- Clignotement 1 Hz : n'avance que lorsque la batterie est vide, la
      -- phase s'inverse toutes les CLIGNO_DEMI_CYCLES_G cycles.
      IF batt_vide_reg = '1' THEN
        IF compteur_cligno = CLIGNO_DEMI_CYCLES_G - 1 THEN
          compteur_cligno <= 0;
          phase_cligno    <= NOT phase_cligno;
        ELSE
          compteur_cligno <= compteur_cligno + 1;
        END IF;
      END IF;

      -- LED7 : clignotement prioritaire sur la PWM quand la batterie est vide.
      IF batt_vide_reg = '1' THEN
        led7_reg <= phase_cligno;
      ELSE
        led7_reg <= pwm_led;
      END IF;

    END IF;
  END PROCESS surveillance;

  ------------------------------------------------------------------------------
  -- Générateur PWM de LED7 (même module que pour les moteurs).
  ------------------------------------------------------------------------------
  u_pwm_gen : pwm_gen
    GENERIC MAP (PERIODE_CYCLES => PERIODE_PWM_G)
    PORT MAP (clk => clk, rst_b => rst_b, duty => duty_led, pwm => pwm_led);

  -- Sorties : copies directes des bascules.
  led7      <= led7_reg;
  batt_vide <= batt_vide_reg;

END ARCHITECTURE batterie_monitor_arch;
