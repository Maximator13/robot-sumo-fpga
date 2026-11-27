--------------------------------------------------------------------------------
-- Fichier      : suiveur_ligne.vhd
-- Entité       : suiveur_ligne
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Commande des deux moteurs en mode « suiveur de ligne » pour
--                la course sur la piste EMA (cahier des charges §4) : circuit
--                ouvert, départ posé à une extrémité de la ligne noire,
--                2 allers/retours, puis arrêt sur le point de départ. Le
--                robot rencontre donc 4 extrémités de ligne (l'extrémité de
--                départ ne compte pas) : demi-tour aux 3 premières, arrêt
--                définitif à la 4e.
-- Principe     : Machine à états de Moore (style du cours, chapitre 16),
--                horloge système 500 kHz, processus synchrone unique :
--                - INACTIF : mode suiveur non sélectionné, moteurs arrêtés,
--                  compteur d'extrémités et mémoire de côté remis à zéro.
--                - SUIVI : correction par paliers selon les 5 capteurs sol.
--                  La correction choisie est mémorisée dans un registre :
--                  elle donne la commande des roues et retient le dernier
--                  côté où la ligne a été vue.
--                - PERTE_LIGNE : les 5 capteurs voient blanc. On tourne vers
--                  le dernier côté vu (virage serré) ; si la ligne était
--                  centrée, on avance tout droit lentement et on compte les
--                  cycles tout blancs CONSÉCUTIFS : FIN_LIGNE_CYCLES_G
--                  cycles signifient une extrémité de ligne.
--                - DEMI_TOUR : pivot sur place, toujours vers la droite,
--                  jusqu'à ce que le capteur centre revoie la ligne.
--                - FIN_PARCOURS : NB_FINS_G extrémités atteintes, arrêt.
--                - ARRET_BATTERIE : batterie vide, arrêt définitif.
--                Priorités : reset > batt_vide > actif = '0' > le reste.
--                Les sorties sont décodées depuis l'état et le registre de
--                correction uniquement (Moore : aucun chemin combinatoire
--                des capteurs vers les moteurs).
-- Entrées      : clk       : horloge système 500 kHz.
--                rst_b     : reset asynchrone actif à '0'.
--                actif     : '1' quand le mode suiveur est sélectionné.
--                sol_noir  : capteurs sol binarisés, '1' = noir (ligne) ;
--                            bit 4 = gauche, 3 = centre gauche, 2 = centre,
--                            1 = centre droit, 0 = droit.
--                batt_vide : '1' quand la batterie est déclarée vide.
-- Sorties      : dir_gauche, dir_droit   : sens de chaque roue
--                                          (DIR_AVANT_* = marche avant).
--                duty_gauche, duty_droit : vitesse de chaque roue en %
--                                          (0 quand le robot est arrêté).
--                etat_dbg  : état courant pour la simulation et Signal Tap :
--                            INACTIF "000", SUIVI "001", PERTE_LIGNE "010",
--                            DEMI_TOUR "011", FIN_PARCOURS "100",
--                            ARRET_BATTERIE "101".
-- Génériques   : FIN_LIGNE_CYCLES_G : nombre de cycles tout blancs
--                consécutifs qui signale une extrémité de ligne (défaut
--                FIN_LIGNE_CYCLES = 100 000, soit 200 ms à 500 kHz, au moins
--                2) ; NB_FINS_G : nombre d'extrémités avant l'arrêt (défaut
--                NB_FINS_LIGNE = 4, soit 2 allers/retours).
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY suiveur_ligne IS
  GENERIC (
    FIN_LIGNE_CYCLES_G : NATURAL := FIN_LIGNE_CYCLES;   -- cycles tout blancs = extrémité
    NB_FINS_G          : NATURAL := NB_FINS_LIGNE       -- extrémités avant l'arrêt
  );
  PORT (
    clk         : IN  STD_LOGIC;                      -- horloge système 500 kHz
    rst_b       : IN  STD_LOGIC;                      -- reset asynchrone, actif à '0'
    actif       : IN  STD_LOGIC;                      -- mode suiveur sélectionné
    sol_noir    : IN  STD_LOGIC_VECTOR(4 DOWNTO 0);   -- capteurs sol, '1' = noir
    batt_vide   : IN  STD_LOGIC;                      -- batterie vide
    dir_gauche  : OUT STD_LOGIC;                      -- sens roue gauche
    dir_droit   : OUT STD_LOGIC;                      -- sens roue droite
    duty_gauche : OUT t_duty;                         -- vitesse roue gauche (%)
    duty_droit  : OUT t_duty;                         -- vitesse roue droite (%)
    etat_dbg    : OUT STD_LOGIC_VECTOR(2 DOWNTO 0)    -- état courant (mise au point)
  );
END ENTITY suiveur_ligne;

ARCHITECTURE suiveur_ligne_arch OF suiveur_ligne IS

  -- États de la machine (codage laissé au synthétiseur).
  TYPE t_etat_suiveur IS (INACTIF, SUIVI, PERTE_LIGNE, DEMI_TOUR, FIN_PARCOURS, ARRET_BATTERIE);
  SIGNAL etat : t_etat_suiveur;

  -- Mouvements possibles du robot (commande des deux roues) :
  --   TOUT_DROIT    : deux roues en avant à VITESSE_SUIVI ;
  --   VIRAGE_GAUCHE : roue gauche ralentie à VITESSE_VIRAGE (virage doux) ;
  --   PIVOT_GAUCHE  : roue gauche en arrière, droite en avant (sur place) ;
  --   AVANCE_LENTE  : deux roues en avant à VITESSE_VIRAGE ;
  --   ARRET         : moteurs arrêtés ; et les symétriques à droite.
  TYPE t_mouvement IS (ARRET, TOUT_DROIT, VIRAGE_GAUCHE, VIRAGE_DROIT,
                       PIVOT_GAUCHE, PIVOT_DROIT, AVANCE_LENTE);

  -- Correction choisie en SUIVI d'après les capteurs. Elle reste mémorisée
  -- pendant une perte de ligne : elle indique alors le dernier côté vu
  -- (TOUT_DROIT = ligne centrée).
  SIGNAL correction : t_mouvement;

  -- Cycles tout blancs consécutifs vus en PERTE_LIGNE (saturé). Il n'est lu
  -- que dans PERTE_LIGNE et repart de 1 à chaque entrée dans cet état : il
  -- n'a pas besoin d'être remis à zéro ailleurs.
  SIGNAL duree_blanc : NATURAL RANGE 0 TO FIN_LIGNE_CYCLES_G;

  -- Extrémités de ligne atteintes depuis l'activation du mode.
  SIGNAL nb_fins : NATURAL RANGE 0 TO NB_FINS_G;

  -- Mouvement commandé dans l'état courant.
  SIGNAL mouvement : t_mouvement;

  -- Vitesses converties au format de la consigne PWM.
  CONSTANT DUTY_SUIVI  : t_duty := TO_UNSIGNED(VITESSE_SUIVI, t_duty'LENGTH);
  CONSTANT DUTY_VIRAGE : t_duty := TO_UNSIGNED(VITESSE_VIRAGE, t_duty'LENGTH);
  CONSTANT DUTY_NUL    : t_duty := (OTHERS => '0');

  -- Correction par paliers d'après la position de la ligne sous les
  -- capteurs, des capteurs extérieurs vers le centre :
  --   ligne sous un capteur extérieur seul côté gauche : pivot à gauche ;
  --   sous le centre gauche (sans le centre droit) : virage doux à gauche ;
  --   symétrique à droite ;
  --   ligne centrée ou vue de façon symétrique (centre seul, croisement
  --   tout noir...) : tout droit.
  FUNCTION correction_capteurs (s : STD_LOGIC_VECTOR(4 DOWNTO 0)) RETURN t_mouvement IS
  BEGIN
    IF s(4) = '1' AND s(0) = '0' THEN
      RETURN PIVOT_GAUCHE;
    ELSIF s(0) = '1' AND s(4) = '0' THEN
      RETURN PIVOT_DROIT;
    ELSIF s(3) = '1' AND s(1) = '0' THEN
      RETURN VIRAGE_GAUCHE;
    ELSIF s(1) = '1' AND s(3) = '0' THEN
      RETURN VIRAGE_DROIT;
    ELSE
      RETURN TOUT_DROIT;
    END IF;
  END FUNCTION correction_capteurs;

BEGIN

  -- Calcul de l'état futur, de la correction, du décompte des cycles blancs
  -- et du compteur d'extrémités (processus synchrone unique).
  sequencement : PROCESS (clk, rst_b)
  BEGIN
    IF rst_b = '0' THEN
      -- Reset : robot à l'arrêt, parcours à recommencer depuis le départ.
      etat        <= INACTIF;
      correction  <= TOUT_DROIT;
      duree_blanc <= 0;
      nb_fins     <= 0;
    ELSIF RISING_EDGE(clk) THEN
      IF etat = ARRET_BATTERIE OR batt_vide = '1' THEN
        -- Batterie vide : arrêt définitif jusqu'au reset, même hors mode
        -- suiveur (priorité sur actif).
        etat <= ARRET_BATTERIE;

      ELSIF actif = '0' THEN
        -- Mode suiveur quitté : arrêt, et la prochaine activation repart
        -- pour un parcours complet (compteur d'extrémités à 0, ligne
        -- supposée centrée).
        etat       <= INACTIF;
        correction <= TOUT_DROIT;
        nb_fins    <= 0;

      ELSE
        CASE etat IS

          WHEN INACTIF =>
            -- Mode suiveur sélectionné : le robot, posé sur la ligne au
            -- point de départ, commence à la suivre.
            etat <= SUIVI;

          WHEN SUIVI =>
            IF sol_noir = "00000" THEN
              -- Plus aucun capteur sur la ligne : premier cycle blanc. La
              -- correction n'est pas modifiée, elle garde le dernier côté vu.
              etat        <= PERTE_LIGNE;
              duree_blanc <= 1;
            ELSE
              correction <= correction_capteurs(sol_noir);
            END IF;

          WHEN PERTE_LIGNE =>
            IF sol_noir /= "00000" THEN
              -- Ligne retrouvée : reprise du suivi, décompte abandonné.
              etat       <= SUIVI;
              correction <= correction_capteurs(sol_noir);
            ELSIF duree_blanc + 1 >= FIN_LIGNE_CYCLES_G AND correction = TOUT_DROIT THEN
              -- Ligne perdue alors qu'elle était centrée, et tout blanc
              -- depuis FIN_LIGNE_CYCLES_G cycles : extrémité de ligne.
              -- La dernière extrémité (retour au départ) arrête le robot.
              IF nb_fins + 1 >= NB_FINS_G THEN
                nb_fins <= NB_FINS_G;
                etat    <= FIN_PARCOURS;
              ELSE
                nb_fins <= nb_fins + 1;
                etat    <= DEMI_TOUR;
              END IF;
            ELSIF duree_blanc < FIN_LIGNE_CYCLES_G THEN
              -- Encore blanc : on continue de compter (virage serré si la
              -- ligne était sur un côté, sans conclure à une extrémité).
              duree_blanc <= duree_blanc + 1;
            END IF;

          WHEN DEMI_TOUR =>
            -- On pivote jusqu'à ce que le capteur centre retrouve la ligne.
            IF sol_noir(2) = '1' THEN
              etat       <= SUIVI;
              correction <= correction_capteurs(sol_noir);
            END IF;

          WHEN FIN_PARCOURS =>
            -- Parcours terminé : on reste arrêté sur le point de départ.
            etat <= FIN_PARCOURS;

          WHEN ARRET_BATTERIE =>
            etat <= ARRET_BATTERIE;

          WHEN OTHERS =>
            etat <= INACTIF;

        END CASE;
      END IF;
    END IF;
  END PROCESS sequencement;

  -- Mouvement de Moore, décodé depuis l'état et la correction mémorisée :
  -- en perte de ligne, on tourne sur place vers le dernier côté vu, ou on
  -- avance lentement si la ligne était centrée (extrémité possible).
  mouvement <= ARRET         WHEN etat = INACTIF OR etat = FIN_PARCOURS OR etat = ARRET_BATTERIE ELSE
               PIVOT_DROIT   WHEN etat = DEMI_TOUR ELSE
               correction    WHEN etat = SUIVI ELSE
               AVANCE_LENTE  WHEN correction = TOUT_DROIT ELSE
               PIVOT_GAUCHE  WHEN correction = VIRAGE_GAUCHE OR correction = PIVOT_GAUCHE ELSE
               PIVOT_DROIT;

  -- Sens des roues : seule la roue intérieure d'un pivot recule.
  dir_gauche <= NOT DIR_AVANT_GAUCHE WHEN mouvement = PIVOT_GAUCHE ELSE DIR_AVANT_GAUCHE;
  dir_droit  <= NOT DIR_AVANT_DROIT  WHEN mouvement = PIVOT_DROIT  ELSE DIR_AVANT_DROIT;

  -- Vitesses : la roue intérieure d'un virage doux est ralentie, les deux
  -- roues sont lentes en avance prudente, sinon vitesse de suivi.
  WITH mouvement SELECT
    duty_gauche <= DUTY_NUL    WHEN ARRET,
                   DUTY_VIRAGE WHEN VIRAGE_GAUCHE | AVANCE_LENTE,
                   DUTY_SUIVI  WHEN OTHERS;

  WITH mouvement SELECT
    duty_droit <= DUTY_NUL    WHEN ARRET,
                  DUTY_VIRAGE WHEN VIRAGE_DROIT | AVANCE_LENTE,
                  DUTY_SUIVI  WHEN OTHERS;

  -- État courant codé pour la mise au point.
  WITH etat SELECT
    etat_dbg <= "000" WHEN INACTIF,
                "001" WHEN SUIVI,
                "010" WHEN PERTE_LIGNE,
                "011" WHEN DEMI_TOUR,
                "100" WHEN FIN_PARCOURS,
                "101" WHEN ARRET_BATTERIE,
                "111" WHEN OTHERS;

END ARCHITECTURE suiveur_ligne_arch;
