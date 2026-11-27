--------------------------------------------------------------------------------
-- Fichier      : fsm_combat.vhd
-- Entité       : fsm_combat
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Machine à états du sous-système "Combat Sumo" imposée par le
--                cahier des charges §6.3. Elle fixe le sens et la vitesse de
--                chaque roue selon la situation (attente, recherche de
--                l'adversaire, attaque, recul au bord du Dohyo, arrêt).
-- Principe     : Machine de Moore, style du cours (chapitre 16, exemple
--                feuCarrefour) : un processus synchrone calcule l'état
--                suivant et le compteur d'attente ; les sorties sont décodées
--                de l'état par affectations concurrentes.
--                États : POWER_ON, ATTENTE_5S, RECHERCHE_ADVERSAIRE,
--                ATTAQUE_ADVERSAIRE, EVITEMENT_BORDURE, ARRET.
--                Priorités, de la plus forte à la plus faible :
--                1. reset (rst_b = '0') : POWER_ON ;
--                2. batterie vide : ARRET, définitif jusqu'au reset ;
--                3. mode combat quitté par le PC (actif = '0') : POWER_ON ;
--                4. transitions du diagramme. La bordure l'emporte toujours
--                   sur l'adversaire : rester sur le Dohyo d'abord.
-- Entrées      : clk, rst_b : horloge 500 kHz, reset asynchrone actif à '0'.
--                actif      : '1' quand le mode combat est sélectionné.
--                adversaire : '1' si un capteur infrarouge voit l'adversaire.
--                bordure    : '1' au bord du Dohyo (maintenu pour le recul).
--                batt_vide  : '1' quand la batterie est vide.
-- Sorties      : avance_gauche, avance_droite   : '1' = la roue avance,
--                                                 '0' = elle recule.
--                vitesse_gauche, vitesse_droite : vitesse de chaque roue en %.
-- Génériques   : ATTENTE_CYCLES_G : attente avant le combat
--                (2 500 000 cycles = 5 s à 500 kHz).
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY fsm_combat IS
  GENERIC (
    ATTENTE_CYCLES_G : NATURAL := 2_500_000   -- 5 s à 500 kHz
  );
  PORT (
    clk            : IN  STD_LOGIC;              -- horloge système 500 kHz
    rst_b          : IN  STD_LOGIC;              -- reset asynchrone, actif à '0'
    actif          : IN  STD_LOGIC;              -- mode combat sélectionné
    adversaire     : IN  STD_LOGIC;              -- adversaire vu
    bordure        : IN  STD_LOGIC;              -- bord du Dohyo détecté
    batt_vide      : IN  STD_LOGIC;              -- batterie vide
    avance_gauche  : OUT STD_LOGIC;              -- roue gauche : '1' avance
    avance_droite  : OUT STD_LOGIC;              -- roue droite : '1' avance
    vitesse_gauche : OUT UNSIGNED(6 DOWNTO 0);   -- vitesse roue gauche (%)
    vitesse_droite : OUT UNSIGNED(6 DOWNTO 0)    -- vitesse roue droite (%)
  );
END ENTITY fsm_combat;

ARCHITECTURE fsm_combat_arch OF fsm_combat IS

  -- États du diagramme du cahier des charges (codage laissé au synthétiseur).
  TYPE t_etat_combat IS (POWER_ON, ATTENTE_5S, RECHERCHE_ADVERSAIRE,
                         ATTAQUE_ADVERSAIRE, EVITEMENT_BORDURE, ARRET);
  SIGNAL etat : t_etat_combat;

  -- Cycles déjà passés dans ATTENTE_5S.
  SIGNAL duree_attente : NATURAL RANGE 0 TO ATTENTE_CYCLES_G;

  -- Vitesse commune aux deux roues, décodée depuis l'état.
  SIGNAL vitesse : UNSIGNED(6 DOWNTO 0);

  -- Vitesses en % (CALIBRATION).
  CONSTANT VITESSE_ARRET     : UNSIGNED(6 DOWNTO 0) := TO_UNSIGNED(0, 7);
  CONSTANT VITESSE_RECHERCHE : UNSIGNED(6 DOWNTO 0) := TO_UNSIGNED(40, 7);   -- rotation lente
  CONSTANT VITESSE_ATTAQUE   : UNSIGNED(6 DOWNTO 0) := TO_UNSIGNED(100, 7);  -- poussée maximale
  CONSTANT VITESSE_RECUL     : UNSIGNED(6 DOWNTO 0) := TO_UNSIGNED(70, 7);   -- recul au bord

BEGIN

  ------------------------------------------------------------------------------
  -- Calcul de l'état suivant et du compteur d'attente.
  ------------------------------------------------------------------------------
  sequencement : PROCESS (clk, rst_b)
  BEGIN
    IF rst_b = '0' THEN
      -- Reset : robot immobile, en attente de l'ordre de combat.
      etat          <= POWER_ON;
      duree_attente <= 0;
    ELSIF RISING_EDGE(clk) THEN

      -- L'attente de 5 s repart de zéro à chaque entrée dans ATTENTE_5S.
      IF etat /= ATTENTE_5S THEN
        duree_attente <= 0;
      END IF;

      IF etat = ARRET OR batt_vide = '1' THEN
        -- Batterie vide : arrêt définitif, seul le reset en fait sortir.
        etat <= ARRET;
      ELSIF actif = '0' THEN
        -- Le PC a quitté le mode combat : retour au repos.
        etat <= POWER_ON;
      ELSE
        CASE etat IS

          WHEN POWER_ON =>
            -- Mode combat demandé : début du compte à rebours de 5 s.
            etat <= ATTENTE_5S;

          WHEN ATTENTE_5S =>
            -- Le robot ne bouge pas pendant 5 s (règlement), puis cherche.
            IF duree_attente = ATTENTE_CYCLES_G - 1 THEN
              etat <= RECHERCHE_ADVERSAIRE;
            ELSE
              duree_attente <= duree_attente + 1;
            END IF;

          WHEN RECHERCHE_ADVERSAIRE =>
            IF bordure = '1' THEN
              etat <= EVITEMENT_BORDURE;
            ELSIF adversaire = '1' THEN
              etat <= ATTAQUE_ADVERSAIRE;
            END IF;

          WHEN ATTAQUE_ADVERSAIRE =>
            IF bordure = '1' THEN
              etat <= EVITEMENT_BORDURE;
            ELSIF adversaire = '0' THEN
              etat <= RECHERCHE_ADVERSAIRE;
            END IF;

          WHEN EVITEMENT_BORDURE =>
            -- Recul tant que la bordure (maintenue 300 ms) est vue.
            IF bordure = '0' THEN
              etat <= RECHERCHE_ADVERSAIRE;
            END IF;

          WHEN ARRET =>
            etat <= ARRET;

        END CASE;
      END IF;
    END IF;
  END PROCESS sequencement;

  ------------------------------------------------------------------------------
  -- Sorties de Moore, décodées depuis l'état.
  ------------------------------------------------------------------------------
  -- Recherche : rotation sur place (roue gauche en avant, droite en arrière).
  -- Évitement : les deux roues reculent. Sinon : marche avant.
  avance_gauche <= '0' WHEN etat = EVITEMENT_BORDURE ELSE '1';
  avance_droite <= '0' WHEN etat = RECHERCHE_ADVERSAIRE OR etat = EVITEMENT_BORDURE ELSE '1';

  -- Même vitesse sur les deux roues ; moteurs coupés hors combat et en ARRET.
  WITH etat SELECT
    vitesse <= VITESSE_RECHERCHE WHEN RECHERCHE_ADVERSAIRE,
               VITESSE_ATTAQUE   WHEN ATTAQUE_ADVERSAIRE,
               VITESSE_RECUL     WHEN EVITEMENT_BORDURE,
               VITESSE_ARRET     WHEN OTHERS;

  vitesse_gauche <= vitesse;
  vitesse_droite <= vitesse;

END ARCHITECTURE fsm_combat_arch;
