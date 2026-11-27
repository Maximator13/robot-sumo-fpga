--------------------------------------------------------------------------------
-- Fichier      : detection_capteurs.vhd
-- Entité       : detection_capteurs
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Met en forme les informations des capteurs pour les machines
--                à états : sol noir ou clair sous chacun des 5 capteurs de
--                sol, signal "bordure" (le robot arrive au bord du Dohyo),
--                présence d'un adversaire à gauche et à droite (capteurs IR),
--                et code ADC de la tension batterie.
-- Principe     : Cadencé par l'horloge système 500 kHz. Les 8 mesures de
--                l'ADC, produites dans le domaine 50 MHz, sont d'abord
--                recopiées dans un registre 500 kHz : tous les calculs se
--                font ensuite sur des valeurs stables.
--                Sol : un capteur voit du noir si son code dépasse
--                SEUIL_NOIR_G (la réflexion baisse sur le noir, le code ADC
--                monte). Bordure : '1' dès qu'au moins un des 5 capteurs voit
--                clair ; un décompteur la maintient à '1' encore
--                BORDURE_MAINTIEN_CYCLES_G cycles après que les 5 capteurs
--                revoient du noir, ce qui fixe la durée du recul du robot.
--                Toute nouvelle vue du clair relance ce maintien.
--                IR : les sorties des capteurs d'obstacle sont asynchrones ;
--                elles traversent 2 bascules de synchronisation (anti
--                métastabilité), l'inversion actif bas -> actif haut étant
--                faite entre la première et la seconde bascule.
--                Toutes les sorties sont des bascules (pas de parasite vers
--                les machines à états).
-- Entrées      : clk         : horloge système 500 kHz.
--                rst_b       : reset asynchrone actif à '0'.
--                voies       : mesures des 8 voies de l'ADC (domaine 50 MHz).
--                ir_gauche_b : capteur d'obstacle gauche, '0' = obstacle.
--                ir_droit_b  : capteur d'obstacle droit, '0' = obstacle.
-- Sorties      : sol_noir      : '1' = sol noir sous le capteur ; bit 4 =
--                                gauche, 3 = centre gauche, 2 = centre,
--                                1 = centre droit, 0 = droit. "00000" après
--                                reset (rien n'est encore mesuré).
--                bordure       : '1' = au moins un capteur voit clair (ou
--                                l'a vu depuis moins de
--                                BORDURE_MAINTIEN_CYCLES_G cycles). '1'
--                                après reset, par prudence.
--                adv_gauche    : adversaire vu à gauche (actif haut).
--                adv_droite    : adversaire vu à droite (actif haut).
--                adversaire    : adversaire vu d'un côté ou de l'autre.
--                code_batterie : code ADC de la tension batterie (voie
--                                VOIE_BAT), 0 après reset.
-- Génériques   : BORDURE_MAINTIEN_CYCLES_G : maintien de bordure, en cycles
--                de clk (défaut BORDURE_MAINTIEN_CYCLES = 150 000, soit
--                300 ms à 500 kHz).
--                SEUIL_NOIR_G : code ADC au-delà duquel le sol est noir
--                (défaut SEUIL_NOIR = 2048, à calibrer sur le robot).
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY detection_capteurs IS
  GENERIC (
    BORDURE_MAINTIEN_CYCLES_G : NATURAL := BORDURE_MAINTIEN_CYCLES;  -- maintien de bordure (cycles)
    SEUIL_NOIR_G              : NATURAL := SEUIL_NOIR                -- seuil noir / clair (code ADC)
  );
  PORT (
    clk           : IN  STD_LOGIC;                      -- horloge système 500 kHz
    rst_b         : IN  STD_LOGIC;                      -- reset asynchrone, actif à '0'
    voies         : IN  t_voies_adc;                    -- mesures ADC (domaine 50 MHz)
    ir_gauche_b   : IN  STD_LOGIC;                      -- obstacle à gauche, actif à '0'
    ir_droit_b    : IN  STD_LOGIC;                      -- obstacle à droite, actif à '0'
    sol_noir      : OUT STD_LOGIC_VECTOR(4 DOWNTO 0);   -- '1' = noir sous le capteur
    bordure       : OUT STD_LOGIC;                      -- bord du Dohyo détecté
    adv_gauche    : OUT STD_LOGIC;                      -- adversaire à gauche
    adv_droite    : OUT STD_LOGIC;                      -- adversaire à droite
    adversaire    : OUT STD_LOGIC;                      -- adversaire d'un côté ou de l'autre
    code_batterie : OUT t_mot_adc                       -- code ADC de la batterie
  );
END ENTITY detection_capteurs;

ARCHITECTURE detection_capteurs_arch OF detection_capteurs IS

  -- Copie des mesures ADC dans le domaine 500 kHz.
  SIGNAL voies_reg : t_voies_adc;

  -- Couleur du sol sous les 5 capteurs pour des mesures données : '1' = noir
  -- (code au-dessus du seuil). Ordre des bits de sol_noir ; le câblage des
  -- capteurs sur les voies de l'ADC est donné par sumo_pkg. Appelée
  -- seulement sur front d'horloge, une fois les mesures recopiées.
  FUNCTION couleur_sol (v : t_voies_adc) RETURN STD_LOGIC_VECTOR IS
    VARIABLE noir : STD_LOGIC_VECTOR(4 DOWNTO 0);
  BEGIN
    noir := (OTHERS => '0');
    IF v(VOIE_SOL_GAUCHE)        > SEUIL_NOIR_G THEN noir(4) := '1'; END IF;
    IF v(VOIE_SOL_CENTRE_GAUCHE) > SEUIL_NOIR_G THEN noir(3) := '1'; END IF;
    IF v(VOIE_SOL_CENTRE)        > SEUIL_NOIR_G THEN noir(2) := '1'; END IF;
    IF v(VOIE_SOL_CENTRE_DROIT)  > SEUIL_NOIR_G THEN noir(1) := '1'; END IF;
    IF v(VOIE_SOL_DROIT)         > SEUIL_NOIR_G THEN noir(0) := '1'; END IF;
    RETURN noir;
  END FUNCTION couleur_sol;

  -- Bascules des sorties sol et bordure.
  SIGNAL sol_noir_reg : STD_LOGIC_VECTOR(4 DOWNTO 0);
  SIGNAL bordure_reg  : STD_LOGIC;
  -- Cycles de maintien de la bordure restant à faire.
  SIGNAL maintien     : NATURAL RANGE 0 TO BORDURE_MAINTIEN_CYCLES_G;

  -- Synchronisation des capteurs IR : première bascule (niveau brut, actif
  -- bas), puis seconde bascule qui mémorise le niveau inversé (actif haut).
  SIGNAL ir_gauche_sync_b : STD_LOGIC;
  SIGNAL ir_droit_sync_b  : STD_LOGIC;
  SIGNAL adv_gauche_reg   : STD_LOGIC;
  SIGNAL adv_droite_reg   : STD_LOGIC;
  SIGNAL adversaire_reg   : STD_LOGIC;

BEGIN

  ------------------------------------------------------------------------------
  -- Recopie des mesures, couleur du sol et bordure.
  ------------------------------------------------------------------------------
  traitement_sol : PROCESS (clk, rst_b)
    -- Couleur du sol calculée sur les mesures recopiées au cycle précédent.
    VARIABLE noir_calcule : STD_LOGIC_VECTOR(4 DOWNTO 0);
  BEGIN
    IF rst_b = '0' THEN
      -- Reset : aucune mesure connue. Par prudence, le robot se considère au
      -- bord tant qu'il n'a pas vu du noir sous ses 5 capteurs.
      voies_reg    <= (OTHERS => (OTHERS => '0'));
      sol_noir_reg <= (OTHERS => '0');
      bordure_reg  <= '1';
      -- Le maintien est rechargé dès le premier front : les mesures remises
      -- à 0 sont vues comme du clair. Une valeur de reset nulle évite à
      -- Quartus des bascules à mise sous tension au niveau haut.
      maintien     <= 0;
    ELSIF RISING_EDGE(clk) THEN
      -- Mesures du domaine 50 MHz figées pour tout le cycle 500 kHz.
      voies_reg    <= voies;
      -- Couleur du sol, et bordure calculée sur la même couleur pour que les
      -- deux sorties changent au même cycle.
      noir_calcule := couleur_sol(voies_reg);
      sol_noir_reg <= noir_calcule;

      IF noir_calcule /= "11111" THEN
        -- Un capteur voit clair : bord du Dohyo, le maintien repart du début.
        bordure_reg <= '1';
        maintien    <= BORDURE_MAINTIEN_CYCLES_G;
      ELSIF maintien /= 0 THEN
        -- Tout est noir mais le maintien n'est pas écoulé : le recul continue.
        maintien    <= maintien - 1;
      ELSE
        -- Tout est noir depuis assez longtemps : le robot est sur le Dohyo.
        bordure_reg <= '0';
      END IF;
    END IF;
  END PROCESS traitement_sol;

  ------------------------------------------------------------------------------
  -- Capteurs d'obstacle : 2 bascules de synchronisation, inversion de
  -- polarité entre les deux (actif bas en entrée, actif haut en sortie).
  ------------------------------------------------------------------------------
  synchronisation_ir : PROCESS (clk, rst_b)
  BEGIN
    IF rst_b = '0' THEN
      -- Reset : niveau de repos des capteurs, aucun adversaire.
      ir_gauche_sync_b <= '1';
      ir_droit_sync_b  <= '1';
      adv_gauche_reg   <= '0';
      adv_droite_reg   <= '0';
      adversaire_reg   <= '0';
    ELSIF RISING_EDGE(clk) THEN
      -- Première bascule : échantillonne le signal asynchrone.
      ir_gauche_sync_b <= ir_gauche_b;
      ir_droit_sync_b  <= ir_droit_b;
      -- Seconde bascule : valeur stable, passée en actif haut. "adversaire"
      -- est calculé sur les mêmes bascules pour rester cohérent avec les
      -- deux côtés à chaque cycle.
      adv_gauche_reg   <= NOT ir_gauche_sync_b;
      adv_droite_reg   <= NOT ir_droit_sync_b;
      adversaire_reg   <= NOT ir_gauche_sync_b OR NOT ir_droit_sync_b;
    END IF;
  END PROCESS synchronisation_ir;

  -- Sorties : directement les bascules.
  sol_noir      <= sol_noir_reg;
  bordure       <= bordure_reg;
  adv_gauche    <= adv_gauche_reg;
  adv_droite    <= adv_droite_reg;
  adversaire    <= adversaire_reg;
  code_batterie <= voies_reg(VOIE_BAT);

END ARCHITECTURE detection_capteurs_arch;
