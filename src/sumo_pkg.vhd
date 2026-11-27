--------------------------------------------------------------------------------
-- Fichier      : sumo_pkg.vhd
-- Entité       : (paquetage) sumo_pkg
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Regroupe en un seul endroit toutes les constantes et tous les
--                types partagés par les modules du robot : fréquences
--                d'horloge, durées exprimées en nombre de cycles, affectation
--                des voies de l'ADC, seuils de calibration, vitesses moteurs,
--                modes de fonctionnement et caractères de commande série.
-- Principe     : Aucun module ne code de valeur "en dur" : il lit ce
--                paquetage. Les valeurs marquées "CALIBRATION" se règlent sur
--                le robot réel puis on recompile, sans toucher à la logique.
--                Les durées sont données en cycles de l'horloge du module qui
--                les utilise (50 MHz pour le reset, 500 kHz pour le reste).
-- Entrées      : sans objet (paquetage)
-- Sorties      : sans objet (paquetage)
-- Génériques   : sans objet (les modules reprennent ces constantes comme
--                valeurs par défaut de leurs génériques, réduits en simulation)
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

PACKAGE sumo_pkg IS

  ------------------------------------------------------------------------------
  -- Horloges
  ------------------------------------------------------------------------------
  -- Oscillateur de la DE0-Nano (broche PIN_R8).
  CONSTANT CLK_IN_HZ  : NATURAL := 50_000_000;
  -- Horloge système imposée par le cahier des charges (§6.2) : 50 MHz / 100.
  CONSTANT CLK_SYS_HZ : NATURAL := 500_000;

  ------------------------------------------------------------------------------
  -- Durées réelles, en nombre de cycles d'horloge
  ------------------------------------------------------------------------------
  -- Maintien de rst_b à '0' : 1 ms à 50 MHz (CDC §6.1).
  CONSTANT RESET_CYCLES            : NATURAL := 50_000;
  -- Durée d'un bit série à 9600 bauds, horloge 500 kHz : 500 000 / 9600 = 52
  -- (débit réel 9615 bauds, écart 0,16 %, toléré par l'UART).
  CONSTANT UART_CLKS_PER_BIT       : NATURAL := 52;
  -- Période PWM moteurs imposée à 5 kHz (CDC §6) : 500 000 / 5 000 = 100 cycles,
  -- ce qui donne un rapport cyclique réglable par pas de 1 %.
  CONSTANT PWM_PERIODE_CYCLES      : NATURAL := 100;
  -- Attente avant le début du combat : 5 s à 500 kHz (CDC §3.2 et §6.3).
  CONSTANT ATTENTE_COMBAT_CYCLES   : NATURAL := 2_500_000;
  -- Demi-période du clignotement de LED7 batterie vide : 0,5 s (1 Hz, CDC §6.4).
  CONSTANT CLIGNO_DEMI_CYCLES      : NATURAL := 250_000;
  -- Temps pendant lequel la batterie doit rester sous le seuil avant d'être
  -- déclarée vide : 1 s. Évite un arrêt définitif sur une chute de tension
  -- brève (démarrage des moteurs). CALIBRATION.
  CONSTANT FILTRE_BATT_CYCLES      : NATURAL := 500_000;
  -- Maintien du signal "bordure" après sa disparition : 300 ms, ce qui fixe la
  -- durée du recul dans l'état EVITEMENT_BORDURE. CALIBRATION.
  CONSTANT BORDURE_MAINTIEN_CYCLES : NATURAL := 150_000;
  -- Temps "tout blanc" au-delà duquel le suiveur conclut à une extrémité de
  -- ligne : 200 ms. CALIBRATION sur la piste EMA (virages serrés).
  CONSTANT FIN_LIGNE_CYCLES        : NATURAL := 100_000;
  -- 2 allers/retours sur un circuit ouvert = 4 extrémités atteintes (CDC §4.2).
  CONSTANT NB_FINS_LIGNE           : NATURAL := 4;

  ------------------------------------------------------------------------------
  -- ADC128S022 (8 voies, 12 bits, Vref = 3,3 V)
  ------------------------------------------------------------------------------
  -- Mot de conversion d'une voie (0 à 4095).
  SUBTYPE t_mot_adc   IS UNSIGNED(11 DOWNTO 0);
  -- Les 8 voies de l'ADC, indicées comme les entrées Analog_In0 à Analog_In7.
  TYPE    t_voies_adc IS ARRAY (0 TO 7) OF t_mot_adc;

  -- Câblage des capteurs sur les voies (schéma carte_fpga_sumo_V4, connecteur
  -- SV2 relié au JP3 de la DE0-Nano). Voies 6 et 7 non câblées.
  CONSTANT VOIE_BAT               : NATURAL := 0;  -- BAT : tension piles divisée
  CONSTANT VOIE_SOL_CENTRE_GAUCHE : NATURAL := 1;  -- EDGE_CENTER_LEFT
  CONSTANT VOIE_SOL_CENTRE        : NATURAL := 2;  -- EDGE_CENTER
  CONSTANT VOIE_SOL_CENTRE_DROIT  : NATURAL := 3;  -- EDGE_CENTER_RIGHT
  CONSTANT VOIE_SOL_GAUCHE        : NATURAL := 4;  -- EDGE_LEFT
  CONSTANT VOIE_SOL_DROIT         : NATURAL := 5;  -- EDGE_RIGHT

  ------------------------------------------------------------------------------
  -- Calibration capteurs et batterie
  ------------------------------------------------------------------------------
  -- Les capteurs sol renvoient une valeur d'autant plus grande que le sol est
  -- sombre : code > SEUIL_NOIR signifie "sol noir". Valeur de départ à
  -- mi-échelle, à régler avec les LED 0 à 4 (le gris des lignes de départ du
  -- Dohyo doit être classé noir, le beige de la bordure classé clair).
  -- CALIBRATION.
  CONSTANT SEUIL_NOIR   : NATURAL := 2048;
  -- Codes ADC de la batterie : BAT = (Vbat - 0,7 V diode D1) / 3 (pont 20k/10k),
  -- code = BAT x 4096 / 3,3 V.
  CONSTANT BAT_CODE_MAX : NATURAL := 3434;  -- Vbat = 9,0 V : LED7 à 100 %
  CONSTANT BAT_CODE_MIN : NATURAL := 2689;  -- Vbat = 7,2 V : seuil batterie vide
  -- Loi LED7 (CDC §6.4) : rapport cyclique de 10 % à BAT_CODE_MIN jusqu'à
  -- 100 % à BAT_CODE_MAX, soit une pente de 90 / (3434 - 2689) = 90 / 745.
  -- Pour éviter un diviseur matériel, la pente est approchée par
  -- PENTE_LED7_NUM / 2**PENTE_LED7_DECALAGE = 31 / 256 (erreur < 1 point) :
  -- duty = 10 + ((code - BAT_CODE_MIN) x 31) / 256.
  -- À RECALCULER si BAT_CODE_MIN ou BAT_CODE_MAX change (le testbench du
  -- paquetage le vérifie).
  CONSTANT PENTE_LED7_NUM      : NATURAL := 31;
  CONSTANT PENTE_LED7_DECALAGE : NATURAL := 8;

  ------------------------------------------------------------------------------
  -- Moteurs
  ------------------------------------------------------------------------------
  -- Niveau de la sortie "sens" qui fait avancer chaque roue. Les deux moteurs
  -- sont montés en miroir : la polarité peut différer entre gauche et droite.
  -- CALIBRATION (vérifier avec SumoTests.jic).
  CONSTANT DIR_AVANT_GAUCHE : STD_LOGIC := '1';
  CONSTANT DIR_AVANT_DROIT  : STD_LOGIC := '1';

  -- Rapport cyclique PWM en pourcent (0 = moteur arrêté, 100 = pleine vitesse).
  SUBTYPE t_duty IS UNSIGNED(6 DOWNTO 0);
  -- Vitesses de chaque comportement, en pourcent. CALIBRATION.
  CONSTANT VITESSE_ATTAQUE   : NATURAL := 100;  -- poussée maximale sur l'adversaire
  CONSTANT VITESSE_RECHERCHE : NATURAL := 40;   -- rotation sur place, lente pour ne pas rater la cible
  CONSTANT VITESSE_RECUL     : NATURAL := 70;   -- marche arrière pour s'écarter de la bordure
  CONSTANT VITESSE_SUIVI     : NATURAL := 60;   -- ligne droite en suivi de ligne
  CONSTANT VITESSE_VIRAGE    : NATURAL := 25;   -- roue intérieure pendant une correction de trajectoire

  ------------------------------------------------------------------------------
  -- Modes de fonctionnement et commandes série
  ------------------------------------------------------------------------------
  -- Mode choisi par le PC : en attente d'ordre après la mise sous tension,
  -- puis combat SUMO ou suiveur de ligne (CDC §1).
  TYPE t_mode IS (MODE_ATTENTE, MODE_COMBAT, MODE_SUIVEUR);
  -- Codes ASCII des caractères de commande reçus du PC.
  CONSTANT CAR_COMBAT  : STD_LOGIC_VECTOR(7 DOWNTO 0) := x"63";  -- 'c' : passe en combat
  CONSTANT CAR_SUIVEUR : STD_LOGIC_VECTOR(7 DOWNTO 0) := x"73";  -- 's' : passe en suiveur

END PACKAGE sumo_pkg;
