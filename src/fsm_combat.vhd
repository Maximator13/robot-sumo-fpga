--------------------------------------------------------------------------------
-- Fichier      : fsm_combat.vhd
-- Entité       : fsm_combat
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Machine à états du sous-système "Combat Sumo" imposée par le
--                cahier des charges §6.3 : POWER_ON, ATTENTE_5S,
--                RECHERCHE_ADVERSAIRE, ATTAQUE_ADVERSAIRE, EVITEMENT_BORDURE,
--                ARRET. Elle fixe le sens et la vitesse (rapport cyclique PWM)
--                de chaque roue.
-- Principe     : Machine de Moore (style du cours, chapitre 16) : un seul
--                processus synchrone à reset asynchrone calcule l'état futur
--                et le compteur d'attente ; les sorties sont décodées de
--                l'état par affectations concurrentes (aucun latch).
--                Interprétations retenues du diagramme du §6.3 :
--                - le '1' de Power ON -> Attente 5S est la sélection du mode
--                  combat par le PC (actif = '1') ;
--                - transition ajoutée au diagramme : retour à POWER_ON quand
--                  le PC quitte le mode combat (actif = '0') ;
--                - Batt. empty mène toujours à ARRET, y compris depuis
--                  Recherche adversaire et Evitement bordure, où le diagramme
--                  dessine aussi une flèche vers Power ON : les moteurs
--                  restent ainsi coupés jusqu'au reset.
--                Priorités, de la plus forte à la plus faible :
--                1. rst_b = '0' : POWER_ON (asynchrone, seul moyen de quitter
--                   ARRET) ;
--                2. batt_vide = '1' : ARRET depuis tout état, même hors mode
--                   combat (piles trop faibles, moteurs coupés définitivement) ;
--                3. actif = '0' (le PC a quitté le mode combat) : POWER_ON ;
--                4. transitions du diagramme : POWER_ON -> ATTENTE_5S dès que
--                   le mode est actif ; ATTENTE_5S -> RECHERCHE après
--                   ATTENTE_CYCLES_G cycles ; la bordure l'emporte toujours
--                   sur l'adversaire (rester sur le Dohyo d'abord) ; après un
--                   évitement on repasse toujours par la recherche.
--                Le côté où l'adversaire est vu (un seul capteur IR) est
--                mémorisé dans deux bascules mises à jour à chaque front :
--                ainsi aucune sortie ne dépend directement d'une entrée
--                (sorties synchrones, comme l'état).
--                Horloge système 500 kHz.
-- Entrées      : clk        : horloge système 500 kHz.
--                rst_b      : reset asynchrone actif à '0'.
--                actif      : '1' quand le mode choisi est MODE_COMBAT.
--                adversaire : '1' si un capteur IR voit l'adversaire.
--                adv_gauche : '1' si le capteur IR gauche le voit.
--                adv_droite : '1' si le capteur IR droit le voit.
--                bordure    : '1' si un capteur sol voit la bordure claire
--                             (déjà étiré par detection_capteurs, ce qui fixe
--                             la durée du recul).
--                batt_vide  : '1' quand la batterie est déclarée vide.
-- Sorties      : dir_gauche, dir_droit   : sens de chaque roue (DIR_AVANT_*
--                                          pour avancer, l'inverse pour reculer).
--                duty_gauche, duty_droit : rapport cyclique de chaque roue (%).
--                etat_dbg : état courant codé pour la simulation et Signal Tap :
--                POWER_ON "000", ATTENTE_5S "001", RECHERCHE_ADVERSAIRE "010",
--                ATTAQUE_ADVERSAIRE "011", EVITEMENT_BORDURE "100", ARRET "101".
-- Génériques   : ATTENTE_CYCLES_G : durée de l'attente avant combat, en cycles
--                de clk (défaut ATTENTE_COMBAT_CYCLES = 2 500 000, soit 5 s à
--                500 kHz).
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY fsm_combat IS
  GENERIC (
    ATTENTE_CYCLES_G : NATURAL := ATTENTE_COMBAT_CYCLES   -- attente avant combat (cycles)
  );
  PORT (
    clk         : IN  STD_LOGIC;                      -- horloge système 500 kHz
    rst_b       : IN  STD_LOGIC;                      -- reset asynchrone, actif à '0'
    actif       : IN  STD_LOGIC;                      -- mode combat sélectionné
    adversaire  : IN  STD_LOGIC;                      -- adversaire vu (gauche ou droite)
    adv_gauche  : IN  STD_LOGIC;                      -- adversaire vu par le capteur gauche
    adv_droite  : IN  STD_LOGIC;                      -- adversaire vu par le capteur droit
    bordure     : IN  STD_LOGIC;                      -- bordure du Dohyo détectée
    batt_vide   : IN  STD_LOGIC;                      -- batterie déclarée vide
    dir_gauche  : OUT STD_LOGIC;                      -- sens roue gauche
    dir_droit   : OUT STD_LOGIC;                      -- sens roue droite
    duty_gauche : OUT t_duty;                         -- vitesse roue gauche (%)
    duty_droit  : OUT t_duty;                         -- vitesse roue droite (%)
    etat_dbg    : OUT STD_LOGIC_VECTOR(2 DOWNTO 0)    -- état courant (mise au point)
  );
END ENTITY fsm_combat;

ARCHITECTURE fsm_combat_arch OF fsm_combat IS

  -- États du diagramme du CDC §6.3 (codage laissé au synthétiseur).
  TYPE t_etat_combat IS (POWER_ON, ATTENTE_5S, RECHERCHE_ADVERSAIRE,
                         ATTAQUE_ADVERSAIRE, EVITEMENT_BORDURE, ARRET);
  SIGNAL etat : t_etat_combat;

  -- Cycles déjà passés dans ATTENTE_5S.
  SIGNAL duree_attente : NATURAL RANGE 0 TO ATTENTE_CYCLES_G;

  -- Adversaire vu par un seul capteur : côté vers lequel il faut braquer.
  SIGNAL adv_seul_gauche : STD_LOGIC;
  SIGNAL adv_seul_droite : STD_LOGIC;

  -- Vitesses converties au format des rapports cycliques.
  CONSTANT DUTY_ARRET     : t_duty := TO_UNSIGNED(0, t_duty'LENGTH);
  CONSTANT DUTY_ATTAQUE   : t_duty := TO_UNSIGNED(VITESSE_ATTAQUE, t_duty'LENGTH);
  CONSTANT DUTY_RECHERCHE : t_duty := TO_UNSIGNED(VITESSE_RECHERCHE, t_duty'LENGTH);
  CONSTANT DUTY_RECUL     : t_duty := TO_UNSIGNED(VITESSE_RECUL, t_duty'LENGTH);
  CONSTANT DUTY_VIRAGE    : t_duty := TO_UNSIGNED(VITESSE_VIRAGE, t_duty'LENGTH);

BEGIN

  ------------------------------------------------------------------------------
  -- Calcul de l'état futur et du compteur d'attente (processus synchrone unique)
  ------------------------------------------------------------------------------
  sequencement : PROCESS (clk, rst_b)
  BEGIN
    IF rst_b = '0' THEN
      -- Reset : robot immobile, en attente de l'ordre de combat.
      etat            <= POWER_ON;
      duree_attente   <= 0;
      adv_seul_gauche <= '0';
      adv_seul_droite <= '0';
    ELSIF RISING_EDGE(clk) THEN

      -- Mémorise le côté de l'adversaire quand un seul capteur le voit.
      adv_seul_gauche <= adv_gauche AND NOT adv_droite;
      adv_seul_droite <= adv_droite AND NOT adv_gauche;

      -- L'attente de 5 s repart de zéro à chaque entrée dans ATTENTE_5S.
      IF etat /= ATTENTE_5S THEN
        duree_attente <= 0;
      END IF;

      IF etat = ARRET THEN
        -- ARRET est définitif : seul le reset en fait sortir.
        etat <= ARRET;
      ELSIF batt_vide = '1' THEN
        -- Piles trop faibles : arrêt des moteurs depuis n'importe quel état.
        etat <= ARRET;
      ELSIF actif = '0' THEN
        -- Le PC a quitté le mode combat : retour au repos.
        etat <= POWER_ON;
      ELSE
        CASE etat IS

          WHEN POWER_ON =>
            -- Mode combat demandé : début du compte à rebours réglementaire.
            etat <= ATTENTE_5S;

          WHEN ATTENTE_5S =>
            -- Le robot ne doit pas bouger pendant 5 s, puis il cherche.
            IF duree_attente = ATTENTE_CYCLES_G - 1 THEN
              etat <= RECHERCHE_ADVERSAIRE;
            ELSE
              duree_attente <= duree_attente + 1;
            END IF;

          WHEN RECHERCHE_ADVERSAIRE =>
            -- Rester sur le Dohyo prime sur l'attaque.
            IF bordure = '1' THEN
              etat <= EVITEMENT_BORDURE;
            ELSIF adversaire = '1' THEN
              etat <= ATTAQUE_ADVERSAIRE;
            END IF;

          WHEN ATTAQUE_ADVERSAIRE =>
            -- On pousse tant que l'adversaire est vu et le bord pas atteint.
            IF bordure = '1' THEN
              etat <= EVITEMENT_BORDURE;
            ELSIF adversaire = '0' THEN
              etat <= RECHERCHE_ADVERSAIRE;
            END IF;

          WHEN EVITEMENT_BORDURE =>
            -- Recul tant que la bordure (étirée) est vue, puis recherche.
            IF bordure = '0' THEN
              etat <= RECHERCHE_ADVERSAIRE;
            END IF;

          WHEN OTHERS =>
            -- État impossible : retour au repos.
            etat <= POWER_ON;

        END CASE;
      END IF;
    END IF;
  END PROCESS sequencement;

  ------------------------------------------------------------------------------
  -- Sorties de Moore, décodées depuis l'état (et le côté mémorisé en attaque)
  ------------------------------------------------------------------------------
  -- Roue gauche : en avant sauf pendant le recul (en recherche, le robot
  -- tourne sur place vers la droite : roue gauche en avant).
  dir_gauche <= NOT DIR_AVANT_GAUCHE WHEN etat = EVITEMENT_BORDURE ELSE DIR_AVANT_GAUCHE;

  -- Roue droite : en arrière pendant la rotation de recherche et le recul.
  dir_droit <= NOT DIR_AVANT_DROIT WHEN etat = RECHERCHE_ADVERSAIRE OR etat = EVITEMENT_BORDURE
               ELSE DIR_AVANT_DROIT;

  -- Vitesse roue gauche : ralentie en attaque si l'adversaire n'est vu qu'à
  -- gauche, pour braquer vers lui. Moteur coupé hors combat et en ARRET.
  duty_gauche <= DUTY_RECHERCHE WHEN etat = RECHERCHE_ADVERSAIRE ELSE
                 DUTY_VIRAGE    WHEN etat = ATTAQUE_ADVERSAIRE AND adv_seul_gauche = '1' ELSE
                 DUTY_ATTAQUE   WHEN etat = ATTAQUE_ADVERSAIRE ELSE
                 DUTY_RECUL     WHEN etat = EVITEMENT_BORDURE ELSE
                 DUTY_ARRET;

  -- Vitesse roue droite : symétrique de la roue gauche.
  duty_droit <= DUTY_RECHERCHE WHEN etat = RECHERCHE_ADVERSAIRE ELSE
                DUTY_VIRAGE    WHEN etat = ATTAQUE_ADVERSAIRE AND adv_seul_droite = '1' ELSE
                DUTY_ATTAQUE   WHEN etat = ATTAQUE_ADVERSAIRE ELSE
                DUTY_RECUL     WHEN etat = EVITEMENT_BORDURE ELSE
                DUTY_ARRET;

  -- État courant pour la simulation et Signal Tap.
  WITH etat SELECT
    etat_dbg <= "000" WHEN POWER_ON,
                "001" WHEN ATTENTE_5S,
                "010" WHEN RECHERCHE_ADVERSAIRE,
                "011" WHEN ATTAQUE_ADVERSAIRE,
                "100" WHEN EVITEMENT_BORDURE,
                "101" WHEN ARRET,
                "111" WHEN OTHERS;

END ARCHITECTURE fsm_combat_arch;
