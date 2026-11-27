--------------------------------------------------------------------------------
-- Fichier      : tb_sumo_pkg.vhd
-- Entité       : tb_sumo_pkg
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant du paquetage sumo_pkg. Il ne
--                recopie pas les valeurs du paquetage : il les RECALCULE à
--                partir des grandeurs physiques du cahier des charges et des
--                schémas (fréquences, durées, pont diviseur batterie, ADC) et
--                vérifie qu'elles coïncident.
-- Principe     : Un seul processus enchaîne les vérifications. Chaque contrôle
--                passe par la procédure verifier, qui compte les contrôles et
--                les échecs et lève une assertion de sévérité ERROR en cas
--                d'écart. "TEST OK" n'est affiché que si aucun échec n'a eu
--                lieu, sinon "TEST KO" avec une sévérité FAILURE.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE IEEE.MATH_REAL.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY tb_sumo_pkg IS
END ENTITY tb_sumo_pkg;

ARCHITECTURE tb_sumo_pkg_arch OF tb_sumo_pkg IS
BEGIN

  verification : PROCESS
    -- Nombre de contrôles effectués et nombre d'échecs, bilan en fin de test.
    VARIABLE nb_controles : NATURAL := 0;
    VARIABLE nb_echecs    : NATURAL := 0;

    -- Effectue un contrôle. ModelSim continue après une assertion de niveau
    -- ERROR : le compteur nb_echecs empêche d'afficher "TEST OK" à tort.
    PROCEDURE verifier (condition : BOOLEAN; message : STRING) IS
    BEGIN
      nb_controles := nb_controles + 1;
      IF NOT condition THEN
        nb_echecs := nb_echecs + 1;
      END IF;
      ASSERT condition REPORT "ECHEC : " & message SEVERITY ERROR;
    END PROCEDURE verifier;

    -- Code ADC attendu pour une tension batterie donnée (schéma carte V4) :
    -- diode D1 (0,7 V), pont R4 = 20k / R12 = 10k, ADC 12 bits sous 3,3 V.
    FUNCTION code_batterie (vbat : REAL) RETURN INTEGER IS
      CONSTANT V_BAT_ADC : REAL := (vbat - 0.7) * 10.0 / (20.0 + 10.0);
    BEGIN
      RETURN INTEGER(ROUND(V_BAT_ADC * 4096.0 / 3.3));
    END FUNCTION code_batterie;

    -- Rapport cyclique LED7 idéal (CDC §6.4) : 10 % à 7,2 V, 100 % à 9 V.
    FUNCTION duty_ideal (code : NATURAL) RETURN REAL IS
    BEGIN
      RETURN 10.0 + 90.0 * REAL(code - BAT_CODE_MIN) / REAL(BAT_CODE_MAX - BAT_CODE_MIN);
    END FUNCTION duty_ideal;

    -- Rapport cyclique LED7 tel que batterie_monitor le calculera, sans
    -- diviseur matériel, avec la pente approchée du paquetage :
    -- 10 + ((code - min) x PENTE_LED7_NUM) / 2**PENTE_LED7_DECALAGE.
    FUNCTION duty_materiel (code : NATURAL) RETURN NATURAL IS
    BEGIN
      RETURN 10 + ((code - BAT_CODE_MIN) * PENTE_LED7_NUM) / (2 ** PENTE_LED7_DECALAGE);
    END FUNCTION duty_materiel;

    VARIABLE ecart_max : REAL := 0.0;  -- pire écart idéal / matériel sur LED7
    VARIABLE ecart     : REAL;

  BEGIN
    ----------------------------------------------------------------------------
    -- 1. Horloges : 50 MHz divisé par un entier exact (100) pour obtenir 500 kHz.
    ----------------------------------------------------------------------------
    verifier(CLK_IN_HZ = 50_000_000, "CLK_IN_HZ doit valoir 50 MHz (oscillateur DE0-Nano)");
    verifier(CLK_SYS_HZ = 500_000, "CLK_SYS_HZ doit valoir 500 kHz (CDC 6.2)");
    verifier(CLK_IN_HZ MOD CLK_SYS_HZ = 0, "50 MHz / 500 kHz doit être un entier");
    verifier(CLK_IN_HZ / CLK_SYS_HZ = 100, "le diviseur doit diviser par 100");

    ----------------------------------------------------------------------------
    -- 2. Durées : chaque constante correspond à la durée physique annoncée.
    ----------------------------------------------------------------------------
    verifier(RESET_CYCLES = CLK_IN_HZ / 1_000, "reset = 1 ms à 50 MHz (CDC 6.1)");
    verifier(UART_CLKS_PER_BIT = CLK_SYS_HZ / 9_600, "bit série = 500 kHz / 9600 bauds");
    -- Écart de débit réel / nominal inférieur à 2 % (marge d'une UART 8N1).
    verifier(ABS(REAL(CLK_SYS_HZ) / REAL(UART_CLKS_PER_BIT) - 9600.0) / 9600.0 < 0.02,
             "débit série réel trop éloigné de 9600 bauds");
    verifier(CLK_SYS_HZ MOD 5_000 = 0, "5 kHz doit diviser 500 kHz exactement");
    verifier(PWM_PERIODE_CYCLES = CLK_SYS_HZ / 5_000, "période PWM = 5 kHz (CDC 6)");
    verifier(PWM_PERIODE_CYCLES = 100, "période PWM de 100 cycles = pas de 1 %");
    verifier(ATTENTE_COMBAT_CYCLES = 5 * CLK_SYS_HZ, "attente combat = 5 s (CDC 3.2)");
    verifier(CLIGNO_DEMI_CYCLES = CLK_SYS_HZ / 2, "demi-période clignotement = 0,5 s (1 Hz)");
    verifier(FILTRE_BATT_CYCLES = CLK_SYS_HZ, "filtre batterie = 1 s");
    verifier(BORDURE_MAINTIEN_CYCLES = (3 * CLK_SYS_HZ) / 10, "maintien bordure = 300 ms");
    verifier(FIN_LIGNE_CYCLES = CLK_SYS_HZ / 5, "fin de ligne = 200 ms");
    verifier(NB_FINS_LIGNE = 4, "2 allers/retours = 4 extrémités (CDC 4.2)");

    ----------------------------------------------------------------------------
    -- 3. ADC : formats et affectation des voies (schéma carte_fpga_sumo_V4).
    ----------------------------------------------------------------------------
    verifier(t_mot_adc'LENGTH = 12, "mot ADC sur 12 bits (ADC128S022)");
    verifier(t_voies_adc'LENGTH = 8, "8 voies ADC");
    verifier(VOIE_BAT = 0, "BAT sur Analog_In0 (JP3 broche 24)");
    verifier(VOIE_SOL_CENTRE_GAUCHE = 1, "EDGE_CENTER_LEFT sur Analog_In1 (JP3 broche 25)");
    verifier(VOIE_SOL_CENTRE = 2, "EDGE_CENTER sur Analog_In2 (JP3 broche 22)");
    verifier(VOIE_SOL_CENTRE_DROIT = 3, "EDGE_CENTER_RIGHT sur Analog_In3 (JP3 broche 21)");
    verifier(VOIE_SOL_GAUCHE = 4, "EDGE_LEFT sur Analog_In4 (JP3 broche 23)");
    verifier(VOIE_SOL_DROIT = 5, "EDGE_RIGHT sur Analog_In5 (JP3 broche 18)");
    verifier(SEUIL_NOIR > 0 AND SEUIL_NOIR < 4095, "seuil noir dans la plage ADC");

    ----------------------------------------------------------------------------
    -- 4. Batterie : codes ADC recalculés depuis le schéma, et approximation
    --    sans division du rapport cyclique LED7 (erreur maximale 1 point).
    ----------------------------------------------------------------------------
    verifier(BAT_CODE_MAX = code_batterie(9.0), "BAT_CODE_MAX ne correspond pas à 9,0 V");
    verifier(BAT_CODE_MIN = code_batterie(7.2), "BAT_CODE_MIN ne correspond pas à 7,2 V");
    verifier(BAT_CODE_MIN < BAT_CODE_MAX AND BAT_CODE_MAX < 4096, "codes batterie incohérents");
    verifier(duty_materiel(BAT_CODE_MIN) = 10, "LED7 doit valoir 10 % à 7,2 V");
    verifier(duty_materiel(BAT_CODE_MAX) = 100, "LED7 doit valoir 100 % à 9,0 V");
    FOR code IN BAT_CODE_MIN TO BAT_CODE_MAX LOOP
      ecart := ABS(REAL(duty_materiel(code)) - duty_ideal(code));
      IF ecart > ecart_max THEN
        ecart_max := ecart;
      END IF;
    END LOOP;
    verifier(ecart_max <= 1.0, "pente LED7 approchée trop éloignée de la droite du CDC (recalculer PENTE_LED7_*)");
    -- Le produit (code - min) x pente doit tenir dans les 12 bits d'écart + la pente
    -- sans débordement : on borne le résultat intermédiaire à 2**20.
    verifier((BAT_CODE_MAX - BAT_CODE_MIN) * PENTE_LED7_NUM < 2 ** 20, "produit LED7 trop grand");
    REPORT "Ecart max LED7 (idéal / matériel) : " & REAL'IMAGE(ecart_max) & " point(s)" SEVERITY NOTE;

    ----------------------------------------------------------------------------
    -- 5. Moteurs : vitesses représentables dans t_duty et cohérentes.
    ----------------------------------------------------------------------------
    verifier(t_duty'LENGTH = 7 AND 2 ** t_duty'LENGTH > 100, "t_duty doit coder 0 à 100");
    verifier(VITESSE_ATTAQUE <= 100 AND VITESSE_RECHERCHE <= 100 AND VITESSE_RECUL <= 100
             AND VITESSE_SUIVI <= 100 AND VITESSE_VIRAGE <= 100, "vitesse > 100 %");
    verifier(VITESSE_RECHERCHE > 0 AND VITESSE_RECUL > 0 AND VITESSE_SUIVI > 0
             AND VITESSE_VIRAGE > 0, "une vitesse nulle immobiliserait le robot");
    verifier(VITESSE_VIRAGE < VITESSE_SUIVI, "la roue intérieure doit tourner moins vite");
    verifier(VITESSE_ATTAQUE = 100, "attaque à pleine puissance");
    verifier(DIR_AVANT_GAUCHE = '0' OR DIR_AVANT_GAUCHE = '1', "polarité gauche non binaire");
    verifier(DIR_AVANT_DROIT = '0' OR DIR_AVANT_DROIT = '1', "polarité droite non binaire");

    ----------------------------------------------------------------------------
    -- 6. Modes et commandes série.
    ----------------------------------------------------------------------------
    verifier(t_mode'LEFT = MODE_ATTENTE, "le mode par défaut doit être l'attente");
    -- Ordre fixe : il sert au codage des LED 6 et 5 ("00" attente, "01" combat, "10" suiveur).
    verifier(t_mode'POS(MODE_COMBAT) = 1 AND t_mode'POS(MODE_SUIVEUR) = 2, "ordre des modes modifié");
    verifier(TO_INTEGER(UNSIGNED(CAR_COMBAT)) = CHARACTER'POS('c'), "CAR_COMBAT doit valoir 'c'");
    verifier(TO_INTEGER(UNSIGNED(CAR_SUIVEUR)) = CHARACTER'POS('s'), "CAR_SUIVEUR doit valoir 's'");

    -- Bilan : "TEST OK" uniquement si aucun contrôle n'a échoué.
    IF nb_echecs = 0 THEN
      REPORT "TEST OK : tb_sumo_pkg (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_sumo_pkg (" & INTEGER'IMAGE(nb_echecs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    std.env.finish;
  END PROCESS verification;

END ARCHITECTURE tb_sumo_pkg_arch;
