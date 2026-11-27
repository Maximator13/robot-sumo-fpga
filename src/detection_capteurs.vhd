--------------------------------------------------------------------------------
-- Fichier      : detection_capteurs.vhd
-- Entité       : detection_capteurs
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Transforme les mesures des capteurs en informations simples
--                pour les machines à états : sol noir ou clair sous chacun
--                des 5 capteurs de sol, bord du Dohyo ("bordure"), présence
--                d'un adversaire (capteurs infrarouges), tension batterie.
-- Principe     : Horloge système 500 kHz.
--                - Les mesures de l'ADC arrivent du domaine 50 MHz : elles
--                  sont d'abord recopiées dans des registres 500 kHz, pour
--                  travailler sur des valeurs stables.
--                - Sol : un capteur voit du noir si son code dépasse
--                  SEUIL_NOIR_G (le noir réfléchit peu, le code ADC monte).
--                - Bordure : '1' dès qu'un des 5 capteurs voit clair (bord
--                  beige du Dohyo). Un décompteur la maintient à '1' encore
--                  BORDURE_MAINTIEN_CYCLES_G cycles après le retour au noir :
--                  c'est la durée du recul du robot.
--                - Adversaire : les capteurs infrarouges (sortie '0' si
--                  obstacle) sont asynchrones ; ils passent par 2 bascules de
--                  synchronisation, puis sont combinés en un seul signal.
--                Toutes les sorties sont des bascules.
-- Entrées      : clk, rst_b      : horloge 500 kHz, reset asynchrone actif à '0'.
--                ch_batterie     : mesure ADC de la batterie (voie 0).
--                ch_sol_*        : mesures ADC des 5 capteurs de sol.
--                ir_gauche_b, ir_droit_b : capteurs d'obstacle, '0' = obstacle.
-- Sorties      : sol_noir      : '1' = noir ; bit 4 = gauche, 3 = centre
--                                gauche, 2 = centre, 1 = centre droit, 0 = droit.
--                bordure       : bord du Dohyo détecté (maintenu).
--                adversaire    : adversaire vu par au moins un capteur IR.
--                code_batterie : code ADC de la batterie (0 à 4095).
-- Génériques   : BORDURE_MAINTIEN_CYCLES_G : maintien de la bordure en
--                cycles (150 000 = 300 ms à 500 kHz).
--                SEUIL_NOIR_G : seuil noir / clair en code ADC (2048, à
--                calibrer sur le robot).
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY detection_capteurs IS
  GENERIC (
    BORDURE_MAINTIEN_CYCLES_G : NATURAL := 150_000;   -- 300 ms à 500 kHz
    SEUIL_NOIR_G              : NATURAL := 2048       -- CALIBRATION
  );
  PORT (
    clk                 : IN  STD_LOGIC;                       -- horloge système 500 kHz
    rst_b               : IN  STD_LOGIC;                       -- reset asynchrone, actif à '0'
    ch_batterie         : IN  STD_LOGIC_VECTOR(11 DOWNTO 0);   -- voie 0 : batterie
    ch_sol_centre_gauche: IN  STD_LOGIC_VECTOR(11 DOWNTO 0);   -- voie 1
    ch_sol_centre       : IN  STD_LOGIC_VECTOR(11 DOWNTO 0);   -- voie 2
    ch_sol_centre_droit : IN  STD_LOGIC_VECTOR(11 DOWNTO 0);   -- voie 3
    ch_sol_gauche       : IN  STD_LOGIC_VECTOR(11 DOWNTO 0);   -- voie 4
    ch_sol_droit        : IN  STD_LOGIC_VECTOR(11 DOWNTO 0);   -- voie 5
    ir_gauche_b         : IN  STD_LOGIC;                       -- obstacle à gauche si '0'
    ir_droit_b          : IN  STD_LOGIC;                       -- obstacle à droite si '0'
    sol_noir            : OUT STD_LOGIC_VECTOR(4 DOWNTO 0);    -- '1' = noir sous le capteur
    bordure             : OUT STD_LOGIC;                       -- bord du Dohyo détecté
    adversaire          : OUT STD_LOGIC;                       -- adversaire vu
    code_batterie       : OUT UNSIGNED(11 DOWNTO 0)            -- tension batterie (code ADC)
  );
END ENTITY detection_capteurs;

ARCHITECTURE detection_capteurs_arch OF detection_capteurs IS

  -- Copies des mesures dans le domaine 500 kHz.
  SIGNAL batterie_reg      : UNSIGNED(11 DOWNTO 0);
  SIGNAL gauche_reg        : UNSIGNED(11 DOWNTO 0);
  SIGNAL centre_gauche_reg : UNSIGNED(11 DOWNTO 0);
  SIGNAL centre_reg        : UNSIGNED(11 DOWNTO 0);
  SIGNAL centre_droit_reg  : UNSIGNED(11 DOWNTO 0);
  SIGNAL droit_reg         : UNSIGNED(11 DOWNTO 0);

  -- Sorties sol et bordure, et cycles de maintien restants.
  SIGNAL sol_noir_reg : STD_LOGIC_VECTOR(4 DOWNTO 0);
  SIGNAL bordure_reg  : STD_LOGIC;
  SIGNAL maintien     : NATURAL RANGE 0 TO BORDURE_MAINTIEN_CYCLES_G;

  -- Synchronisation des capteurs IR (1re bascule, niveau brut).
  SIGNAL ir_gauche_sync_b : STD_LOGIC;
  SIGNAL ir_droit_sync_b  : STD_LOGIC;
  SIGNAL adversaire_reg   : STD_LOGIC;

BEGIN

  ------------------------------------------------------------------------------
  -- Recopie des mesures, couleur du sol et bordure.
  ------------------------------------------------------------------------------
  traitement_sol : PROCESS (clk, rst_b)
    VARIABLE noir : STD_LOGIC_VECTOR(4 DOWNTO 0);   -- couleur calculée ce cycle
  BEGIN
    IF rst_b = '0' THEN
      -- Reset : aucune mesure. Par prudence, le robot se considère au bord
      -- tant qu'il n'a pas vu du noir sous ses 5 capteurs.
      batterie_reg      <= (OTHERS => '0');
      gauche_reg        <= (OTHERS => '0');
      centre_gauche_reg <= (OTHERS => '0');
      centre_reg        <= (OTHERS => '0');
      centre_droit_reg  <= (OTHERS => '0');
      droit_reg         <= (OTHERS => '0');
      sol_noir_reg      <= (OTHERS => '0');
      bordure_reg       <= '1';
      maintien          <= 0;
    ELSIF RISING_EDGE(clk) THEN
      -- Mesures du domaine 50 MHz figées pour tout le cycle.
      batterie_reg      <= UNSIGNED(ch_batterie);
      gauche_reg        <= UNSIGNED(ch_sol_gauche);
      centre_gauche_reg <= UNSIGNED(ch_sol_centre_gauche);
      centre_reg        <= UNSIGNED(ch_sol_centre);
      centre_droit_reg  <= UNSIGNED(ch_sol_centre_droit);
      droit_reg         <= UNSIGNED(ch_sol_droit);

      -- Couleur du sol sous chaque capteur : '1' = noir.
      noir := "00000";
      IF gauche_reg        > SEUIL_NOIR_G THEN noir(4) := '1'; END IF;
      IF centre_gauche_reg > SEUIL_NOIR_G THEN noir(3) := '1'; END IF;
      IF centre_reg        > SEUIL_NOIR_G THEN noir(2) := '1'; END IF;
      IF centre_droit_reg  > SEUIL_NOIR_G THEN noir(1) := '1'; END IF;
      IF droit_reg         > SEUIL_NOIR_G THEN noir(0) := '1'; END IF;
      sol_noir_reg <= noir;

      -- Bordure, calculée sur la même couleur.
      IF noir /= "11111" THEN
        -- Un capteur voit clair : bord du Dohyo, le maintien repart.
        bordure_reg <= '1';
        maintien    <= BORDURE_MAINTIEN_CYCLES_G;
      ELSIF maintien /= 0 THEN
        -- Tout est noir mais le maintien continue (le robot recule encore).
        maintien    <= maintien - 1;
      ELSE
        -- Tout est noir depuis assez longtemps : le robot est sur le Dohyo.
        bordure_reg <= '0';
      END IF;
    END IF;
  END PROCESS traitement_sol;

  ------------------------------------------------------------------------------
  -- Capteurs d'obstacle : 2 bascules de synchronisation (anti-métastabilité).
  ------------------------------------------------------------------------------
  synchronisation_ir : PROCESS (clk, rst_b)
  BEGIN
    IF rst_b = '0' THEN
      ir_gauche_sync_b <= '1';
      ir_droit_sync_b  <= '1';
      adversaire_reg   <= '0';
    ELSIF RISING_EDGE(clk) THEN
      -- 1re bascule : échantillonne les signaux asynchrones.
      ir_gauche_sync_b <= ir_gauche_b;
      ir_droit_sync_b  <= ir_droit_b;
      -- 2e bascule : adversaire vu si l'un des deux capteurs est à '0'.
      adversaire_reg   <= NOT ir_gauche_sync_b OR NOT ir_droit_sync_b;
    END IF;
  END PROCESS synchronisation_ir;

  -- Sorties : directement les bascules.
  sol_noir      <= sol_noir_reg;
  bordure       <= bordure_reg;
  adversaire    <= adversaire_reg;
  code_batterie <= batterie_reg;

END ARCHITECTURE detection_capteurs_arch;
