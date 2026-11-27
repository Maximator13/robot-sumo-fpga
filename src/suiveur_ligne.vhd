--------------------------------------------------------------------------------
-- Fichier      : suiveur_ligne.vhd
-- Entité       : suiveur_ligne
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Commande des moteurs en mode « suiveur de ligne » pour la
--                course sur la piste EMA (cahier des charges §4) : circuit
--                ouvert, départ à une extrémité de la ligne noire, 2 allers /
--                retours puis arrêt au point de départ. Le robot rencontre
--                donc 4 extrémités : demi-tour aux 3 premières, arrêt à la 4e.
-- Principe     : Machine à états de Moore (style du cours, chapitre 16),
--                horloge 500 kHz :
--                - INACTIF : mode non sélectionné, moteurs arrêtés ;
--                - SUIVI : correction selon la position de la ligne sous les
--                  5 capteurs (tout droit, correction légère, virage doux, roue intérieure
--                  arrêtée pour un virage serré, sans marche arrière) ;
--                - PERTE_LIGNE : les 5 capteurs voient blanc. Si la ligne
--                  était sur un côté, c'est un virage serré : on pivote de ce
--                  côté. Si elle était au centre, on avance lentement ; après
--                  FIN_LIGNE_CYCLES_G cycles de blanc, c'est une extrémité ;
--                - DEMI_TOUR : pivot à droite jusqu'à retrouver la ligne ;
--                - FIN_PARCOURS : 4 extrémités atteintes, arrêt ;
--                - ARRET_BATTERIE : batterie vide, arrêt définitif.
--                Le registre "position" mémorise les derniers capteurs ayant
--                vu la ligne : il donne la correction en SUIVI et le côté où
--                tourner en PERTE_LIGNE.
--                Angle droit : la branche passe sous un capteur extérieur
--                juste avant la perte de ligne. Ce côté est retenu pendant
--                FIN_LIGNE_CYCLES_G cycles (branche_g / branche_d) : la perte
--                de ligne qui suit donne un pivot de ce côté, jamais un
--                demi-tour.
-- Entrées      : clk, rst_b : horloge 500 kHz, reset asynchrone actif à '0'.
--                actif      : '1' quand le mode suiveur est sélectionné.
--                sol_noir   : capteurs de sol, '1' = noir ; bit 4 = gauche,
--                             3 = centre gauche, 2 = centre, 1 = centre
--                             droit, 0 = droit.
--                batt_vide  : '1' quand la batterie est vide.
-- Sorties      : avance_gauche, avance_droite   : '1' = la roue avance.
--                vitesse_gauche, vitesse_droite : vitesse de chaque roue en %.
-- Génériques   : FIN_LIGNE_CYCLES_G : cycles de blanc qui signalent une
--                extrémité (100 000 = 200 ms à 500 kHz, à régler sur la piste).
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY suiveur_ligne IS
  GENERIC (
    FIN_LIGNE_CYCLES_G : NATURAL := 100_000   -- 200 ms à 500 kHz (CALIBRATION)
  );
  PORT (
    clk            : IN  STD_LOGIC;                      -- horloge système 500 kHz
    rst_b          : IN  STD_LOGIC;                      -- reset asynchrone, actif à '0'
    actif          : IN  STD_LOGIC;                      -- mode suiveur sélectionné
    sol_noir       : IN  STD_LOGIC_VECTOR(4 DOWNTO 0);   -- capteurs sol, '1' = noir
    batt_vide      : IN  STD_LOGIC;                      -- batterie vide
    avance_gauche  : OUT STD_LOGIC;                      -- roue gauche : '1' avance
    avance_droite  : OUT STD_LOGIC;                      -- roue droite : '1' avance
    vitesse_gauche : OUT UNSIGNED(6 DOWNTO 0);           -- vitesse roue gauche (%)
    vitesse_droite : OUT UNSIGNED(6 DOWNTO 0)            -- vitesse roue droite (%)
  );
END ENTITY suiveur_ligne;

ARCHITECTURE suiveur_ligne_arch OF suiveur_ligne IS

  -- États de la machine (codage laissé au synthétiseur).
  TYPE t_etat_suiveur IS (INACTIF, SUIVI, PERTE_LIGNE, DEMI_TOUR,
                          FIN_PARCOURS, ARRET_BATTERIE);
  SIGNAL etat : t_etat_suiveur;

  -- 2 allers / retours sur un circuit ouvert = 4 extrémités atteintes.
  CONSTANT NB_EXTREMITES : NATURAL := 4;

  -- Vitesses en % (CALIBRATION).
  CONSTANT VITESSE_ARRET  : UNSIGNED(6 DOWNTO 0) := TO_UNSIGNED(0, 7);
  CONSTANT VITESSE_SUIVI  : UNSIGNED(6 DOWNTO 0) := TO_UNSIGNED(65, 7);  -- ligne droite
  CONSTANT VITESSE_LEGER  : UNSIGNED(6 DOWNTO 0) := TO_UNSIGNED(45, 7);  -- roue intérieure, léger écart
  CONSTANT VITESSE_VIRAGE : UNSIGNED(6 DOWNTO 0) := TO_UNSIGNED(30, 7);  -- roue intérieure, avance lente

  -- Derniers capteurs ayant vu la ligne (même ordre que sol_noir).
  SIGNAL position    : STD_LOGIC_VECTOR(4 DOWNTO 0);
  -- Cycles de blanc consécutifs en PERTE_LIGNE.
  SIGNAL duree_blanc : NATURAL RANGE 0 TO FIN_LIGNE_CYCLES_G;
  -- Extrémités atteintes depuis l'activation du mode.
  SIGNAL nb_fins     : NATURAL RANGE 0 TO NB_EXTREMITES;
  -- Branche d'angle droit vue récemment par un capteur extérieur, et
  -- cycles restants avant de l'oublier (décompte gelé en PERTE_LIGNE).
  SIGNAL branche_g     : STD_LOGIC;
  SIGNAL branche_d     : STD_LOGIC;
  SIGNAL duree_branche : NATURAL RANGE 0 TO FIN_LIGNE_CYCLES_G;

  -- Position de la ligne déduite de "position", du plus fort au plus
  -- faible écart : capteur extérieur seul (loin), capteur centre-côté sans
  -- le centre (pres), capteur centre-côté avec le centre (leger).
  SIGNAL ligne_a_gauche_loin  : STD_LOGIC;
  SIGNAL ligne_a_droite_loin  : STD_LOGIC;
  SIGNAL ligne_a_gauche_pres  : STD_LOGIC;
  SIGNAL ligne_a_droite_pres  : STD_LOGIC;
  SIGNAL ligne_a_gauche_leger : STD_LOGIC;
  SIGNAL ligne_a_droite_leger : STD_LOGIC;
  -- Ligne perdue : côté où pivoter (branche récente, sinon dernière position).
  SIGNAL perdue_a_gauche      : STD_LOGIC;
  SIGNAL perdue_a_droite      : STD_LOGIC;

  -- Mouvements commandés, décodés depuis l'état et la position.
  SIGNAL pivot_gauche  : STD_LOGIC;   -- roue gauche recule, droite avance
  SIGNAL pivot_droit   : STD_LOGIC;   -- roue droite recule, gauche avance
  SIGNAL ralentir_g    : STD_LOGIC;   -- roue gauche à vitesse réduite
  SIGNAL ralentir_d    : STD_LOGIC;   -- roue droite à vitesse réduite
  SIGNAL corriger_g    : STD_LOGIC;   -- roue gauche à vitesse légère
  SIGNAL corriger_d    : STD_LOGIC;   -- roue droite à vitesse légère
  SIGNAL arreter_g     : STD_LOGIC;   -- roue gauche arrêtée (virage serré)
  SIGNAL arreter_d     : STD_LOGIC;   -- roue droite arrêtée (virage serré)
  SIGNAL moteurs_coupes: STD_LOGIC;   -- robot à l'arrêt

BEGIN

  ------------------------------------------------------------------------------
  -- Calcul de l'état suivant, de la position, du décompte de blanc et du
  -- nombre d'extrémités.
  ------------------------------------------------------------------------------
  sequencement : PROCESS (clk, rst_b)
  BEGIN
    IF rst_b = '0' THEN
      etat        <= INACTIF;
      position    <= "00100";   -- ligne supposée centrée
      duree_blanc <= 0;
      nb_fins     <= 0;
      branche_g     <= '0';
      branche_d     <= '0';
      duree_branche <= 0;
    ELSIF RISING_EDGE(clk) THEN

      -- Mémorise la position de la ligne chaque fois qu'elle est vue.
      IF sol_noir /= "00000" THEN
        position <= sol_noir;
      END IF;

      -- Branche vue par un seul capteur extérieur : côté retenu un moment.
      IF sol_noir(4) = '1' AND sol_noir(0) = '0' THEN
        branche_g     <= '1';
        branche_d     <= '0';
        duree_branche <= FIN_LIGNE_CYCLES_G;
      ELSIF sol_noir(0) = '1' AND sol_noir(4) = '0' THEN
        branche_g     <= '0';
        branche_d     <= '1';
        duree_branche <= FIN_LIGNE_CYCLES_G;
      ELSIF etat /= PERTE_LIGNE THEN
        IF duree_branche > 0 THEN
          duree_branche <= duree_branche - 1;
        ELSE
          branche_g <= '0';
          branche_d <= '0';
        END IF;
      END IF;

      IF etat = ARRET_BATTERIE OR batt_vide = '1' THEN
        -- Batterie vide : arrêt définitif jusqu'au reset.
        etat <= ARRET_BATTERIE;

      ELSIF actif = '0' THEN
        -- Mode quitté : la prochaine activation repart pour un parcours
        -- complet, ligne supposée centrée.
        etat     <= INACTIF;
        position <= "00100";
        nb_fins  <= 0;
        branche_g <= '0';
        branche_d <= '0';

      ELSE
        CASE etat IS

          WHEN INACTIF =>
            -- Le robot, posé sur la ligne au départ, commence à la suivre.
            etat <= SUIVI;

          WHEN SUIVI =>
            -- Plus aucun capteur sur la ligne : premier cycle de blanc.
            IF sol_noir = "00000" THEN
              etat        <= PERTE_LIGNE;
              duree_blanc <= 1;
            END IF;

          WHEN PERTE_LIGNE =>
            IF sol_noir /= "00000" THEN
              -- Ligne retrouvée : reprise du suivi.
              etat <= SUIVI;
            ELSIF duree_blanc >= FIN_LIGNE_CYCLES_G - 1
                  AND perdue_a_gauche = '0' AND perdue_a_droite = '0' THEN
              -- Blanc prolongé alors que la ligne était au centre, sans
              -- branche d'angle droit vue juste avant : extrémité de ligne. La 4e (retour au départ) arrête le robot.
              IF nb_fins = NB_EXTREMITES - 1 THEN
                nb_fins <= NB_EXTREMITES;
                etat    <= FIN_PARCOURS;
              ELSE
                nb_fins <= nb_fins + 1;
                etat    <= DEMI_TOUR;
              END IF;
            ELSIF duree_blanc < FIN_LIGNE_CYCLES_G THEN
              duree_blanc <= duree_blanc + 1;
            END IF;

          WHEN DEMI_TOUR =>
            -- Pivot jusqu'à ce que le capteur centre retrouve la ligne.
            IF sol_noir(2) = '1' THEN
              etat <= SUIVI;
            END IF;

          WHEN FIN_PARCOURS =>
            etat <= FIN_PARCOURS;

          WHEN ARRET_BATTERIE =>
            etat <= ARRET_BATTERIE;

        END CASE;
      END IF;
    END IF;
  END PROCESS sequencement;

  ------------------------------------------------------------------------------
  -- Position de la ligne, des capteurs extérieurs vers le centre.
  ------------------------------------------------------------------------------
  ligne_a_gauche_loin  <= position(4) AND NOT position(3) AND NOT position(0);
  ligne_a_droite_loin  <= position(0) AND NOT position(1) AND NOT position(4);
  ligne_a_gauche_pres  <= position(3) AND NOT position(2) AND NOT position(1);
  ligne_a_droite_pres  <= position(1) AND NOT position(2) AND NOT position(3);
  ligne_a_gauche_leger <= position(3) AND position(2) AND NOT position(1);
  ligne_a_droite_leger <= position(1) AND position(2) AND NOT position(3);

  -- Ligne perdue : une branche récente l'emporte sur la dernière position.
  perdue_a_gauche <= branche_g OR (NOT branche_d AND (ligne_a_gauche_loin
                     OR ligne_a_gauche_pres OR ligne_a_gauche_leger));
  perdue_a_droite <= branche_d OR (NOT branche_g AND (ligne_a_droite_loin
                     OR ligne_a_droite_pres OR ligne_a_droite_leger));

  ------------------------------------------------------------------------------
  -- Mouvements de Moore, décodés depuis l'état et la position mémorisée.
  ------------------------------------------------------------------------------
  -- SUIVI : roue intérieure arrêtée si la ligne est sous un capteur extérieur
  --         (pas de marche arrière : pas d'à-coup dans les courbes), virage
  --         doux (roue intérieure ralentie) sous un capteur centre-côté seul,
  --         correction légère s'il est vu avec le centre. Ces paliers
  --         progressifs évitent les saccades dans les courbes.
  -- PERTE_LIGNE : pivot vers la branche ou le dernier côté vu, sinon avance
  --               lente (peut-être une extrémité).
  -- DEMI_TOUR : pivot à droite.
  pivot_gauche <= '1' WHEN (etat = PERTE_LIGNE AND perdue_a_gauche = '1')
                  ELSE '0';
  pivot_droit  <= '1' WHEN (etat = PERTE_LIGNE AND perdue_a_droite = '1')
                        OR etat = DEMI_TOUR
                  ELSE '0';
  ralentir_g   <= '1' WHEN (etat = SUIVI AND ligne_a_gauche_pres = '1')
                        OR (etat = PERTE_LIGNE AND perdue_a_gauche = '0' AND perdue_a_droite = '0')
                  ELSE '0';
  ralentir_d   <= '1' WHEN (etat = SUIVI AND ligne_a_droite_pres = '1')
                        OR (etat = PERTE_LIGNE AND perdue_a_gauche = '0' AND perdue_a_droite = '0')
                  ELSE '0';
  corriger_g   <= '1' WHEN etat = SUIVI AND ligne_a_gauche_leger = '1' ELSE '0';
  corriger_d   <= '1' WHEN etat = SUIVI AND ligne_a_droite_leger = '1' ELSE '0';
  arreter_g    <= '1' WHEN etat = SUIVI AND ligne_a_gauche_loin = '1' ELSE '0';
  arreter_d    <= '1' WHEN etat = SUIVI AND ligne_a_droite_loin = '1' ELSE '0';
  moteurs_coupes <= '1' WHEN etat = INACTIF OR etat = FIN_PARCOURS OR etat = ARRET_BATTERIE
                    ELSE '0';

  -- Sens : seule la roue intérieure d'un pivot recule.
  avance_gauche <= NOT pivot_gauche;
  avance_droite <= NOT pivot_droit;

  -- Vitesses : arrêt, vitesse réduite, vitesse légère, ou vitesse de suivi.
  vitesse_gauche <= VITESSE_ARRET  WHEN moteurs_coupes = '1' OR arreter_g = '1' ELSE
                    VITESSE_VIRAGE WHEN ralentir_g = '1' ELSE
                    VITESSE_LEGER  WHEN corriger_g = '1' ELSE
                    VITESSE_SUIVI;
  vitesse_droite <= VITESSE_ARRET  WHEN moteurs_coupes = '1' OR arreter_d = '1' ELSE
                    VITESSE_VIRAGE WHEN ralentir_d = '1' ELSE
                    VITESSE_LEGER  WHEN corriger_d = '1' ELSE
                    VITESSE_SUIVI;

END ARCHITECTURE suiveur_ligne_arch;
