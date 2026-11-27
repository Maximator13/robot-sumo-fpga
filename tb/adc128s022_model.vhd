--------------------------------------------------------------------------------
-- Fichier      : adc128s022_model.vhd
-- Entité       : adc128s022_model
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Modèle de SIMULATION (non synthétisable, réservé à tb/) du
--                convertisseur ADC128S022 de la DE0-Nano, d'après sa
--                datasheet (docs/DOCs/doc_DE0/Compo_du_DEO_Datasheets).
-- Principe     : - Une trame commence au front descendant de CS et dure un
--                  multiple de 16 fronts montants de SCLK.
--                - La voie convertie est celle dont l'adresse a été reçue
--                  pendant la trame PRÉCÉDENTE (IN0 à la mise sous tension).
--                  Le modèle n'a pas de reset : comme le vrai composant, il
--                  garde son adresse quand le FPGA est remis à zéro.
--                - DOUT : fronts descendants 1 à 4 = zéros, fronts 5 à 16 =
--                  D11 à D0 (poids fort d'abord) ; haute impédance quand CS
--                  est à '1'.
--                - DIN : lu sur les fronts montants 1 à 8 (registre de
--                  contrôle, bit 7 en premier) ; ADD2..ADD0 = bits 5 à 3,
--                  donc fronts montants 3, 4 et 5.
--                - Contrôles de la datasheet, comptés dans erreurs_timing :
--                  SCLK entre 0,8 et 3,2 MHz, rapport cyclique 40 à 60 %
--                  (tCH et tCL >= 0,4 x période), tCSS >= 10 ns (CS avant le
--                  1er front montant), tDS >= 10 ns et tDH >= 10 ns (DIN
--                  autour du front montant), SCLK à '1' au début de la trame
--                  (mode choisi pour le contrôleur du robot).
--                - DOUT vaut 'X' pendant tEN (30 ns) après la descente de CS
--                  et pendant tDACC (27 ns) après chaque front descendant de
--                  SCLK : un contrôleur qui lit DOUT trop tôt lit 'X' et la
--                  mesure est fausse.
--                - Une trame qui ne contient pas un multiple de 16 fronts
--                  montants (trame interrompue) est comptée à part dans
--                  trames_incompletes ; si elle a reçu au moins 5 fronts, la
--                  nouvelle adresse est quand même prise en compte (cas le
--                  plus défavorable pour le contrôleur).
-- Entrées      : cs_n, sclk, din : bus SPI piloté par le contrôleur.
--                valeurs         : valeurs 12 bits des 8 voies, mises bout à
--                                  bout (voie k = bits 12k+11 à 12k).
-- Sorties      : dout              : donnée série vers le contrôleur.
--                nb_conversions    : conversions complètes effectuées.
--                erreurs_timing    : violations des contraintes ci-dessus.
--                trames_incompletes: trames interrompues avant 16 fronts.
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY adc128s022_model IS
  PORT (
    cs_n               : IN  STD_LOGIC;
    sclk               : IN  STD_LOGIC;
    din                : IN  STD_LOGIC;
    dout               : OUT STD_LOGIC;
    valeurs            : IN  STD_LOGIC_VECTOR(95 DOWNTO 0);
    nb_conversions     : OUT NATURAL := 0;
    erreurs_timing     : OUT NATURAL := 0;
    trames_incompletes : OUT NATURAL := 0
  );
END ENTITY adc128s022_model;

ARCHITECTURE adc128s022_model_arch OF adc128s022_model IS
  CONSTANT PERIODE_SCLK_MIN : TIME := 312.5 ns;  -- 3,2 MHz max
  CONSTANT PERIODE_SCLK_MAX : TIME := 1250 ns;   -- 0,8 MHz min
  CONSTANT T_SETUP_HOLD     : TIME := 10 ns;     -- tCSS, tDS, tDH (max des valeurs mini)
  CONSTANT T_DACC           : TIME := 27 ns;     -- DOUT valide au plus tard 27 ns après le front descendant de SCLK
  CONSTANT T_EN             : TIME := 30 ns;     -- DOUT actif au plus tard 30 ns après la descente de CS
  -- Rapport cyclique de SCLK : tCH et tCL >= 0,4 x période (40 % à 60 %).
BEGIN

  modele : PROCESS
    VARIABLE adresse     : NATURAL := 0;   -- voie de la prochaine conversion (IN0 au démarrage)
    VARIABLE mot         : STD_LOGIC_VECTOR(11 DOWNTO 0);   -- résultat de la conversion en cours
    VARIABLE ctrl        : STD_LOGIC_VECTOR(7 DOWNTO 0);
    VARIABLE nb_montants : NATURAL;
    VARIABLE nb_descendants : NATURAL;
    VARIABLE rang        : NATURAL;        -- rang du front dans la conversion (1 à 16)
    VARIABLE t_cs, t_montant : TIME;
    VARIABLE t_descendant : TIME;          -- instant du dernier front descendant de SCLK
    VARIABLE periode      : TIME;          -- dernière période de SCLK mesurée
    VARIABLE conversions, erreurs, incompletes : NATURAL := 0;

    PROCEDURE erreur (message : STRING) IS
    BEGIN
      erreurs := erreurs + 1;
      erreurs_timing <= erreurs;
      REPORT "ADC128S022 (modèle) : " & message SEVERITY WARNING;
    END PROCEDURE erreur;

  BEGIN
    dout <= 'Z';
    WAIT UNTIL FALLING_EDGE(cs_n);
    -- Début de trame : la voie sélectionnée à la trame précédente est convertie.
    t_cs := NOW;
    t_montant := NOW;
    t_descendant := NOW;
    periode := 0 ns;
    mot := valeurs(12 * adresse + 11 DOWNTO 12 * adresse);
    ctrl := (OTHERS => '0');
    nb_montants := 0;
    nb_descendants := 0;
    -- DOUT indéfini tant que tEN n'est pas écoulé (lecture trop précoce = 'X').
    dout <= 'X', '0' AFTER T_EN;
    IF sclk /= '1' THEN
      erreur("SCLK doit être à '1' quand CS descend");
    END IF;

    LOOP
      WAIT ON sclk, cs_n, din;
      EXIT WHEN cs_n = '1';

      IF din'EVENT AND nb_montants > 0 AND NOW - t_montant < T_SETUP_HOLD THEN
        erreur("tDH : DIN change " & TIME'IMAGE(NOW - t_montant) & " après le front montant");
      END IF;

      IF FALLING_EDGE(sclk) THEN
        nb_descendants := nb_descendants + 1;
        rang := ((nb_descendants - 1) MOD 16) + 1;
        -- tCH : durée de la phase haute qui se termine.
        IF nb_montants > 0 AND periode > 0 ns AND NOW - t_montant < periode * 4 / 10 THEN
          erreur("tCH : SCLK haut seulement " & TIME'IMAGE(NOW - t_montant) & " (40 % de la période minimum)");
        END IF;
        t_descendant := NOW;
        -- DOUT n'est garanti que tDACC après le front descendant : entre les
        -- deux, il vaut 'X' (un contrôleur qui lit trop tôt lit 'X').
        IF rang <= 4 THEN
          dout <= 'X', '0' AFTER T_DACC;             -- 4 zéros de tête
        ELSE
          dout <= 'X', mot(16 - rang) AFTER T_DACC;  -- D11 au front 5 ... D0 au front 16
        END IF;

      ELSIF RISING_EDGE(sclk) THEN
        nb_montants := nb_montants + 1;
        rang := ((nb_montants - 1) MOD 16) + 1;
        -- Contraintes temporelles.
        IF nb_montants = 1 AND NOW - t_cs < T_SETUP_HOLD THEN
          erreur("tCSS : CS trop proche du premier front montant");
        END IF;
        IF nb_montants > 1 THEN
          periode := NOW - t_montant;
          -- tCL : durée de la phase basse qui se termine.
          IF NOW - t_descendant < periode * 4 / 10 THEN
            erreur("tCL : SCLK bas seulement " & TIME'IMAGE(NOW - t_descendant) & " (40 % de la période minimum)");
          END IF;
        END IF;
        IF nb_montants > 1 AND NOW - t_montant < PERIODE_SCLK_MIN THEN
          erreur("SCLK trop rapide : période " & TIME'IMAGE(NOW - t_montant));
        END IF;
        IF nb_montants > 1 AND NOW - t_montant > PERIODE_SCLK_MAX THEN
          erreur("SCLK trop lente : période " & TIME'IMAGE(NOW - t_montant));
        END IF;
        IF din'LAST_EVENT < T_SETUP_HOLD THEN
          erreur("tDS : DIN stable seulement " & TIME'IMAGE(din'LAST_EVENT) & " avant le front montant");
        END IF;
        t_montant := NOW;
        -- Registre de contrôle : fronts montants 1 à 8, bit 7 en premier.
        IF rang <= 8 THEN
          ctrl(8 - rang) := din;
        END IF;
        -- Fin d'une conversion de 16 fronts : la nouvelle adresse vaut pour
        -- la conversion suivante (mode continu si CS reste bas).
        IF rang = 16 THEN
          conversions := conversions + 1;
          nb_conversions <= conversions;
          adresse := TO_INTEGER(UNSIGNED(ctrl(5 DOWNTO 3)));
          mot := valeurs(12 * adresse + 11 DOWNTO 12 * adresse);
        END IF;
      END IF;
    END LOOP;

    -- Fin de trame (CS remonte).
    IF nb_montants MOD 16 /= 0 THEN
      incompletes := incompletes + 1;
      trames_incompletes <= incompletes;
      IF (nb_montants MOD 16) >= 5 THEN
        adresse := TO_INTEGER(UNSIGNED(ctrl(5 DOWNTO 3)));   -- cas le plus défavorable
      END IF;
    END IF;
  END PROCESS modele;

END ARCHITECTURE adc128s022_model_arch;
