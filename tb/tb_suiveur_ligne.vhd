--------------------------------------------------------------------------------
-- Fichier      : tb_suiveur_ligne.vhd
-- Entité       : tb_suiveur_ligne
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant du suiveur de ligne (course sur
--                la piste EMA, cahier des charges §4).
-- Principe     : L'horloge 500 kHz est figée à '0' pendant les resets, comme
--                en sortie du diviseur de fréquence sur la carte. Les
--                stimuli changent sur front descendant, les contrôles sont
--                faits 2 ns après un front montant.
--                Instance u_suiveur_ligne (FIN_LIGNE_CYCLES_G réduit à 20,
--                NB_FINS_G = 4 réel) :
--                 1. reset, constantes réelles, INACTIF (duty 0 pour les 32
--                    combinaisons de capteurs) ;
--                 2. SUIVI : commande des roues pour les 31 combinaisons non
--                    blanches (cas nommés de la spécification, puis table de
--                    référence), sorties de Moore (rien ne bouge avant le
--                    front suivant) ;
--                 3. PERTE_LIGNE : pivot vers le dernier côté vu, avance lente
--                    tout droit si la ligne était centrée ; pas d'extrémité
--                    après une perte sur le côté, même longue ;
--                 4. extrémité : exactement FIN_LIGNE_CYCLES_G cycles blancs
--                    CONSÉCUTIFS après une ligne centrée (pas un de moins) ;
--                 5. DEMI_TOUR : pivot à droite tant que le centre est blanc ;
--                 6. INACTIF remet le compteur d'extrémités à 0 ;
--                 7. parcours complet simulé (lignes droites, virages serrés,
--                    croisements, 4 extrémités) : demi-tour aux 3 premières,
--                    FIN_PARCOURS à la 4e et pas avant ; puis désactivation
--                    depuis FIN_PARCOURS et nouveau parcours complet ;
--                 8. priorités : batt_vide depuis chaque état (même inactif,
--                    même avec actif = '0'), ARRET_BATTERIE définitif, actif =
--                    '0' depuis chaque état, reset asynchrone.
--                Instance u_reel (génériques par défaut) : extrémité détectée
--                après exactement 100 000 cycles blancs (200 ms), pas 99 999.
--                Instance u_fins_defaut (seul FIN_LIGNE_CYCLES_G réduit,
--                NB_FINS_G par défaut comme dans robot_sumo_top) : mêmes
--                stimuli que u_suiveur_ligne, elle doit avoir à chaque front
--                le même état et les mêmes commandes moteurs ; contrôle
--                explicite : demi-tour à la 3e extrémité, FIN_PARCOURS à la 4e.
--                Un observateur vérifie à chaque front les invariants (code
--                d'état connu, duty 0 à l'arrêt, pivot à droite en demi-tour)
--                et compte les entrées en DEMI_TOUR et en FIN_PARCOURS.
--                Chien de garde à 2 s. "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY tb_suiveur_ligne IS
END ENTITY tb_suiveur_ligne;

ARCHITECTURE tb_suiveur_ligne_arch OF tb_suiveur_ligne IS

  CONSTANT PERIODE_CLK : TIME    := 2 us;   -- horloge 500 kHz
  CONSTANT FIN_COURT   : NATURAL := 20;     -- FIN_LIGNE_CYCLES_G réduit
  CONSTANT NB_FINS     : NATURAL := 4;      -- valeur réelle (2 allers/retours)

  -- Codes de etat_dbg (identiques à ceux de suiveur_ligne).
  CONSTANT E_INACTIF   : STD_LOGIC_VECTOR(2 DOWNTO 0) := "000";
  CONSTANT E_SUIVI     : STD_LOGIC_VECTOR(2 DOWNTO 0) := "001";
  CONSTANT E_PERTE     : STD_LOGIC_VECTOR(2 DOWNTO 0) := "010";
  CONSTANT E_DEMI_TOUR : STD_LOGIC_VECTOR(2 DOWNTO 0) := "011";
  CONSTANT E_FIN       : STD_LOGIC_VECTOR(2 DOWNTO 0) := "100";
  CONSTANT E_BATTERIE  : STD_LOGIC_VECTOR(2 DOWNTO 0) := "101";

  -- Combinaisons de capteurs nommées (bit 4 = gauche ... bit 0 = droit).
  CONSTANT BLANC         : STD_LOGIC_VECTOR(4 DOWNTO 0) := "00000";
  CONSTANT CENTRE        : STD_LOGIC_VECTOR(4 DOWNTO 0) := "00100";
  CONSTANT CENTRE_GAUCHE : STD_LOGIC_VECTOR(4 DOWNTO 0) := "01000";
  CONSTANT CENTRE_DROIT  : STD_LOGIC_VECTOR(4 DOWNTO 0) := "00010";
  CONSTANT GAUCHE        : STD_LOGIC_VECTOR(4 DOWNTO 0) := "10000";
  CONSTANT DROIT         : STD_LOGIC_VECTOR(4 DOWNTO 0) := "00001";

  -- Mouvements attendus (commande des deux roues).
  TYPE t_mvt IS (M_ARRET, M_TOUT_DROIT, M_VIRAGE_GAUCHE, M_VIRAGE_DROIT,
                 M_PIVOT_GAUCHE, M_PIVOT_DROIT, M_AVANCE_LENTE);

  SIGNAL clk       : STD_LOGIC := '0';
  SIGNAL rst_b     : STD_LOGIC := '0';
  SIGNAL fin_simu  : BOOLEAN   := FALSE;

  -- Instance à génériques réduits.
  SIGNAL actif       : STD_LOGIC := '0';
  SIGNAL sol_noir    : STD_LOGIC_VECTOR(4 DOWNTO 0) := CENTRE;
  SIGNAL batt_vide   : STD_LOGIC := '0';
  SIGNAL dir_gauche  : STD_LOGIC;
  SIGNAL dir_droit   : STD_LOGIC;
  SIGNAL duty_gauche : t_duty;
  SIGNAL duty_droit  : t_duty;
  SIGNAL etat_dbg    : STD_LOGIC_VECTOR(2 DOWNTO 0);

  -- Instance à génériques réels.
  SIGNAL actif_reel    : STD_LOGIC := '0';
  SIGNAL sol_noir_reel : STD_LOGIC_VECTOR(4 DOWNTO 0) := CENTRE;
  SIGNAL dir_g_reel    : STD_LOGIC;
  SIGNAL dir_d_reel    : STD_LOGIC;
  SIGNAL duty_g_reel   : t_duty;
  SIGNAL duty_d_reel   : t_duty;
  SIGNAL etat_reel     : STD_LOGIC_VECTOR(2 DOWNTO 0);

  -- Instance à NB_FINS_G par défaut (mêmes entrées que u_suiveur_ligne).
  SIGNAL dir_g_defaut  : STD_LOGIC;
  SIGNAL dir_d_defaut  : STD_LOGIC;
  SIGNAL duty_g_defaut : t_duty;
  SIGNAL duty_d_defaut : t_duty;
  SIGNAL etat_defaut   : STD_LOGIC_VECTOR(2 DOWNTO 0);

  -- Observateur.
  SIGNAL nb_demi_tours : NATURAL := 0;   -- entrées en DEMI_TOUR
  SIGNAL nb_arrivees   : NATURAL := 0;   -- entrées en FIN_PARCOURS
  SIGNAL erreurs_obs   : NATURAL := 0;

  -- Correction de référence en SUIVI, écrite à partir de la spécification
  -- (des capteurs extérieurs vers le centre).
  FUNCTION mvt_reference (s : STD_LOGIC_VECTOR(4 DOWNTO 0)) RETURN t_mvt IS
    VARIABLE ext_g, ext_d, int_g, int_d : BOOLEAN;
  BEGIN
    ext_g := s(4) = '1';  ext_d := s(0) = '1';
    int_g := s(3) = '1';  int_d := s(1) = '1';
    IF ext_g AND NOT ext_d THEN
      RETURN M_PIVOT_GAUCHE;
    ELSIF ext_d AND NOT ext_g THEN
      RETURN M_PIVOT_DROIT;
    ELSIF int_g AND NOT int_d THEN
      RETURN M_VIRAGE_GAUCHE;
    ELSIF int_d AND NOT int_g THEN
      RETURN M_VIRAGE_DROIT;
    ELSE
      RETURN M_TOUT_DROIT;
    END IF;
  END FUNCTION mvt_reference;

  FUNCTION image (s : STD_LOGIC_VECTOR) RETURN STRING IS
    VARIABLE r : STRING(1 TO s'LENGTH);
    VARIABLE k : POSITIVE := 1;
  BEGIN
    FOR i IN s'RANGE LOOP
      IF s(i) = '1' THEN r(k) := '1'; ELSE r(k) := '0'; END IF;
      k := k + 1;
    END LOOP;
    RETURN r;
  END FUNCTION image;

BEGIN

  u_suiveur_ligne : ENTITY WORK.suiveur_ligne
    GENERIC MAP (FIN_LIGNE_CYCLES_G => FIN_COURT, NB_FINS_G => NB_FINS)
    PORT MAP (clk => clk, rst_b => rst_b, actif => actif, sol_noir => sol_noir,
              batt_vide => batt_vide, dir_gauche => dir_gauche, dir_droit => dir_droit,
              duty_gauche => duty_gauche, duty_droit => duty_droit, etat_dbg => etat_dbg);

  u_reel : ENTITY WORK.suiveur_ligne
    PORT MAP (clk => clk, rst_b => rst_b, actif => actif_reel, sol_noir => sol_noir_reel,
              batt_vide => '0', dir_gauche => dir_g_reel, dir_droit => dir_d_reel,
              duty_gauche => duty_g_reel, duty_droit => duty_d_reel, etat_dbg => etat_reel);

  -- NB_FINS_G laissé à sa valeur par défaut : c'est celle qui tourne sur la
  -- carte (robot_sumo_top ne la surcharge pas).
  u_fins_defaut : ENTITY WORK.suiveur_ligne
    GENERIC MAP (FIN_LIGNE_CYCLES_G => FIN_COURT)
    PORT MAP (clk => clk, rst_b => rst_b, actif => actif, sol_noir => sol_noir,
              batt_vide => batt_vide, dir_gauche => dir_g_defaut, dir_droit => dir_d_defaut,
              duty_gauche => duty_g_defaut, duty_droit => duty_d_defaut, etat_dbg => etat_defaut);

  -- Horloge 500 kHz comme en sortie du diviseur de fréquence : '0' pendant le
  -- reset, premier front montant une demi-période après le relâchement.
  horloge : PROCESS
  BEGIN
    WHILE NOT fin_simu LOOP
      IF rst_b = '0' THEN
        clk <= '0';
        WAIT UNTIL rst_b = '1';
        WAIT FOR PERIODE_CLK / 2;
      END IF;
      clk <= '1';
      WAIT UNTIL rst_b = '0' FOR PERIODE_CLK / 2;
      clk <= '0';
      IF rst_b = '1' THEN
        WAIT UNTIL rst_b = '0' FOR PERIODE_CLK / 2;
      END IF;
    END LOOP;
    WAIT;
  END PROCESS horloge;

  -- Invariants vérifiés à chaque front : code d'état connu, moteurs arrêtés
  -- dans INACTIF, FIN_PARCOURS et ARRET_BATTERIE, pivot à droite à vitesse de
  -- suivi en DEMI_TOUR, u_fins_defaut identique à u_suiveur_ligne (même
  -- nombre d'extrémités par défaut que la valeur réelle). Compte aussi les entrées en DEMI_TOUR et FIN_PARCOURS.
  observateur : PROCESS
    VARIABLE precedent : STD_LOGIC_VECTOR(2 DOWNTO 0) := E_INACTIF;
  BEGIN
    WAIT UNTIL RISING_EDGE(clk);
    WAIT FOR 1 ns;
    IF etat_dbg /= E_INACTIF AND etat_dbg /= E_SUIVI AND etat_dbg /= E_PERTE AND
       etat_dbg /= E_DEMI_TOUR AND etat_dbg /= E_FIN AND etat_dbg /= E_BATTERIE THEN
      erreurs_obs <= erreurs_obs + 1;
      REPORT "ECHEC : code d'état inconnu " & image(etat_dbg) SEVERITY ERROR;
    END IF;
    IF (etat_dbg = E_INACTIF OR etat_dbg = E_FIN OR etat_dbg = E_BATTERIE) AND
       (duty_gauche /= 0 OR duty_droit /= 0) THEN
      erreurs_obs <= erreurs_obs + 1;
      REPORT "ECHEC : moteurs non arrêtés dans l'état " & image(etat_dbg) SEVERITY ERROR;
    END IF;
    IF etat_dbg = E_DEMI_TOUR AND NOT (dir_gauche = DIR_AVANT_GAUCHE AND dir_droit = NOT DIR_AVANT_DROIT AND
                                       duty_gauche = VITESSE_SUIVI AND duty_droit = VITESSE_SUIVI) THEN
      erreurs_obs <= erreurs_obs + 1;
      REPORT "ECHEC : le demi-tour n'est pas un pivot à droite" SEVERITY ERROR;
    END IF;
    IF etat_defaut /= etat_dbg OR dir_g_defaut /= dir_gauche OR dir_d_defaut /= dir_droit OR
       duty_g_defaut /= duty_gauche OR duty_d_defaut /= duty_droit THEN
      erreurs_obs <= erreurs_obs + 1;
      REPORT "ECHEC : NB_FINS_G par défaut différent de 4 (u_fins_defaut en " & image(etat_defaut) &
             ", u_suiveur_ligne en " & image(etat_dbg) & ")" SEVERITY ERROR;
    END IF;
    IF etat_dbg = E_DEMI_TOUR AND precedent /= E_DEMI_TOUR THEN
      nb_demi_tours <= nb_demi_tours + 1;
    END IF;
    IF etat_dbg = E_FIN AND precedent /= E_FIN THEN
      nb_arrivees <= nb_arrivees + 1;
    END IF;
    precedent := etat_dbg;
  END PROCESS observateur;

  chien_de_garde : PROCESS
  BEGIN
    WAIT FOR 2 sec;
    IF NOT fin_simu THEN
      REPORT "TEST KO : tb_suiveur_ligne bloqué (délai de 2 s dépassé)" SEVERITY FAILURE;
    END IF;
    WAIT;
  END PROCESS chien_de_garde;

  verification : PROCESS
    VARIABLE nb_controles : NATURAL := 0;
    VARIABLE nb_echecs    : NATURAL := 0;
    VARIABLE s            : STD_LOGIC_VECTOR(4 DOWNTO 0);
    VARIABLE mvt_avant    : t_mvt;
    VARIABLE demi_avant   : NATURAL;

    PROCEDURE verifier (condition : BOOLEAN; message : STRING) IS
    BEGIN
      nb_controles := nb_controles + 1;
      IF NOT condition THEN
        nb_echecs := nb_echecs + 1;
      END IF;
      ASSERT condition REPORT "ECHEC : " & message SEVERITY ERROR;
    END PROCEDURE verifier;

    -- Attend n fronts montants, puis 2 ns (sorties stabilisées et compteurs
    -- de l'observateur, mis à jour 1 ns après le front, à jour).
    PROCEDURE fronts (n : NATURAL) IS
    BEGIN
      FOR i IN 1 TO n LOOP
        WAIT UNTIL RISING_EDGE(clk);
      END LOOP;
      WAIT FOR 2 ns;
    END PROCEDURE fronts;

    -- Présente une combinaison de capteurs (sur front descendant) puis la
    -- laisse n fronts montants.
    PROCEDURE appliquer (c : STD_LOGIC_VECTOR(4 DOWNTO 0); n : NATURAL) IS
    BEGIN
      WAIT UNTIL FALLING_EDGE(clk);
      sol_noir <= c;
      fronts(n);
    END PROCEDURE appliquer;

    PROCEDURE poser_actif (v : STD_LOGIC) IS
    BEGIN
      WAIT UNTIL FALLING_EDGE(clk);
      actif <= v;
    END PROCEDURE poser_actif;

    PROCEDURE poser_batt (v : STD_LOGIC) IS
    BEGIN
      WAIT UNTIL FALLING_EDGE(clk);
      batt_vide <= v;
    END PROCEDURE poser_batt;

    -- Reset de 1 ms (horloge figée), puis 2 fronts.
    PROCEDURE reset_complet IS
    BEGIN
      WAIT UNTIL FALLING_EDGE(clk);
      rst_b <= '0';
      WAIT FOR 1 ms;
      rst_b <= '1';
      fronts(2);
    END PROCEDURE reset_complet;

    -- Vérifie les commandes des deux roues pour un mouvement attendu.
    PROCEDURE verifier_mvt (m : t_mvt; contexte : STRING) IS
      VARIABLE av_g, av_d : BOOLEAN;
      VARIABLE v_g, v_d   : NATURAL;
    BEGIN
      av_g := TRUE; av_d := TRUE;
      CASE m IS
        WHEN M_ARRET         => v_g := 0;              v_d := 0;
        WHEN M_TOUT_DROIT    => v_g := VITESSE_SUIVI;  v_d := VITESSE_SUIVI;
        WHEN M_VIRAGE_GAUCHE => v_g := VITESSE_VIRAGE; v_d := VITESSE_SUIVI;
        WHEN M_VIRAGE_DROIT  => v_g := VITESSE_SUIVI;  v_d := VITESSE_VIRAGE;
        WHEN M_PIVOT_GAUCHE  => v_g := VITESSE_SUIVI;  v_d := VITESSE_SUIVI; av_g := FALSE;
        WHEN M_PIVOT_DROIT   => v_g := VITESSE_SUIVI;  v_d := VITESSE_SUIVI; av_d := FALSE;
        WHEN M_AVANCE_LENTE  => v_g := VITESSE_VIRAGE; v_d := VITESSE_VIRAGE;
      END CASE;
      verifier(TO_INTEGER(duty_gauche) = v_g, contexte & " : duty gauche " &
               INTEGER'IMAGE(TO_INTEGER(duty_gauche)) & " au lieu de " & INTEGER'IMAGE(v_g) &
               " (" & t_mvt'IMAGE(m) & ")");
      verifier(TO_INTEGER(duty_droit) = v_d, contexte & " : duty droit " &
               INTEGER'IMAGE(TO_INTEGER(duty_droit)) & " au lieu de " & INTEGER'IMAGE(v_d) &
               " (" & t_mvt'IMAGE(m) & ")");
      IF m /= M_ARRET THEN
        verifier((dir_gauche = DIR_AVANT_GAUCHE) = av_g, contexte & " : mauvais sens roue gauche (" & t_mvt'IMAGE(m) & ")");
        verifier((dir_droit = DIR_AVANT_DROIT) = av_d, contexte & " : mauvais sens roue droite (" & t_mvt'IMAGE(m) & ")");
      END IF;
    END PROCEDURE verifier_mvt;

    PROCEDURE verifier_etat (e : STD_LOGIC_VECTOR(2 DOWNTO 0); contexte : STRING) IS
    BEGIN
      verifier(etat_dbg = e, contexte & " : état " & image(etat_dbg) & " au lieu de " & image(e));
    END PROCEDURE verifier_etat;

    -- Depuis SUIVI, ligne centrée : FIN_LIGNE_CYCLES_G cycles blancs
    -- consécutifs (extrémité), puis demi-tour à droite jusqu'à ce que le
    -- centre revoie la ligne, et reprise du suivi tout droit.
    PROCEDURE extremite_et_demi_tour (contexte : STRING) IS
    BEGIN
      appliquer(CENTRE, 3);
      verifier_etat(E_SUIVI, contexte & " : avant l'extrémité");
      appliquer(BLANC, FIN_COURT - 1);
      verifier_etat(E_PERTE, contexte & " : extrémité trop tôt");
      verifier_mvt(M_AVANCE_LENTE, contexte & " : approche de l'extrémité");
      fronts(1);
      verifier_etat(E_DEMI_TOUR, contexte & " : extrémité non détectée");
      -- Demi-tour : blanc, puis la ligne passe sous les capteurs de droite.
      appliquer(BLANC, 15);
      appliquer(DROIT, 3);
      appliquer("00011", 2);
      appliquer(CENTRE_DROIT, 2);
      verifier_etat(E_DEMI_TOUR, contexte & " : demi-tour arrêté avant le centre");
      appliquer(CENTRE, 1);
      verifier_etat(E_SUIVI, contexte & " : pas de reprise du suivi après le demi-tour");
      verifier_mvt(M_TOUT_DROIT, contexte & " : reprise après le demi-tour");
    END PROCEDURE extremite_et_demi_tour;

    -- Morceau de piste : ligne droite avec dérives, croisement tout noir,
    -- virages doux, et virage serré (ligne perdue sur le côté bien plus
    -- longtemps que FIN_LIGNE_CYCLES_G : ce n'est pas une extrémité).
    PROCEDURE troncon (contexte : STRING) IS
    BEGIN
      appliquer(CENTRE, 10);
      appliquer("01100", 4);
      appliquer(CENTRE_GAUCHE, 4);
      verifier_mvt(M_VIRAGE_GAUCHE, contexte & " : virage doux à gauche");
      appliquer("11111", 2);
      verifier_mvt(M_TOUT_DROIT, contexte & " : croisement tout noir");
      appliquer("00110", 3);
      appliquer(CENTRE_DROIT, 3);
      -- Virage serré à droite : la ligne sort par la droite.
      appliquer(DROIT, 2);
      appliquer(BLANC, 3 * FIN_COURT);
      verifier_etat(E_PERTE, contexte & " : virage serré à droite");
      verifier_mvt(M_PIVOT_DROIT, contexte & " : virage serré à droite");
      appliquer(DROIT, 2);
      appliquer("00011", 2);
      appliquer(CENTRE, 5);
      -- Virage serré à gauche (sortie par le centre gauche).
      appliquer(CENTRE_GAUCHE, 2);
      appliquer(BLANC, 2 * FIN_COURT);
      verifier_mvt(M_PIVOT_GAUCHE, contexte & " : virage serré à gauche");
      appliquer("11000", 2);
      appliquer(CENTRE, 5);
      verifier_etat(E_SUIVI, contexte & " : fin de tronçon");
    END PROCEDURE troncon;

  BEGIN
    ----------------------------------------------------------------------------
    -- 1. Reset (horloge figée), valeurs réelles, INACTIF.
    ----------------------------------------------------------------------------
    verifier(FIN_LIGNE_CYCLES = 100_000, "FIN_LIGNE_CYCLES doit valoir 200 ms à 500 kHz");
    verifier(NB_FINS_LIGNE = 4, "NB_FINS_LIGNE doit valoir 4 (2 allers/retours)");
    verifier(VITESSE_SUIVI /= VITESSE_VIRAGE AND VITESSE_VIRAGE > 0,
             "vitesses de suivi et de virage indiscernables par le banc");
    actif <= '1';
    WAIT FOR 500 us;
    verifier_etat(E_INACTIF, "pendant le reset");
    verifier_mvt(M_ARRET, "pendant le reset");
    actif <= '0';
    WAIT FOR 500 us;
    rst_b <= '1';
    fronts(2);
    verifier_etat(E_INACTIF, "après le reset, mode non sélectionné");
    -- Inactif : moteurs arrêtés quels que soient les capteurs.
    FOR v IN 0 TO 31 LOOP
      appliquer(STD_LOGIC_VECTOR(TO_UNSIGNED(v, 5)), 1);
      verifier_etat(E_INACTIF, "inactif, capteurs " & INTEGER'IMAGE(v));
      verifier_mvt(M_ARRET, "inactif, capteurs " & INTEGER'IMAGE(v));
    END LOOP;

    ----------------------------------------------------------------------------
    -- 2. SUIVI : commande des roues pour chaque combinaison de capteurs.
    ----------------------------------------------------------------------------
    appliquer(CENTRE, 1);
    poser_actif('1');
    fronts(1);
    verifier_etat(E_SUIVI, "activation du mode suiveur");
    fronts(1);
    verifier_mvt(M_TOUT_DROIT, "centre seul");
    -- Cas nommés de la spécification.
    appliquer(CENTRE_GAUCHE, 1);  verifier_mvt(M_VIRAGE_GAUCHE, "centre gauche seul");
    appliquer(GAUCHE, 1);         verifier_mvt(M_PIVOT_GAUCHE, "gauche seul");
    appliquer(CENTRE, 1);         verifier_mvt(M_TOUT_DROIT, "centre seul");
    appliquer(CENTRE_DROIT, 1);   verifier_mvt(M_VIRAGE_DROIT, "centre droit seul");
    appliquer(DROIT, 1);          verifier_mvt(M_PIVOT_DROIT, "droit seul");
    appliquer("01100", 1);        verifier_mvt(M_VIRAGE_GAUCHE, "centre et centre gauche");
    appliquer("00110", 1);        verifier_mvt(M_VIRAGE_DROIT, "centre et centre droit");
    appliquer("11000", 1);        verifier_mvt(M_PIVOT_GAUCHE, "gauche et centre gauche");
    appliquer("00011", 1);        verifier_mvt(M_PIVOT_DROIT, "droit et centre droit");
    appliquer("01110", 1);        verifier_mvt(M_TOUT_DROIT, "trois capteurs centraux");
    appliquer("11111", 1);        verifier_mvt(M_TOUT_DROIT, "tout noir (croisement)");
    -- Les 31 combinaisons non blanches, deux fois (depuis deux mouvements
    -- précédents différents). Moore : la nouvelle combinaison présentée sur
    -- front descendant ne change rien avant le front montant suivant.
    mvt_avant := M_TOUT_DROIT;
    FOR passe IN 0 TO 1 LOOP
      FOR v IN 1 TO 31 LOOP
        IF passe = 0 THEN
          s := STD_LOGIC_VECTOR(TO_UNSIGNED(v, 5));
        ELSE
          s := STD_LOGIC_VECTOR(TO_UNSIGNED(32 - v, 5));
        END IF;
        WAIT UNTIL FALLING_EDGE(clk);
        sol_noir <= s;
        WAIT FOR 1 ns;
        verifier_mvt(mvt_avant, "Moore : sortie modifiée sans front d'horloge (" & image(s) & ")");
        fronts(1);
        verifier_etat(E_SUIVI, "capteurs " & image(s));
        verifier_mvt(mvt_reference(s), "capteurs " & image(s));
        mvt_avant := mvt_reference(s);
      END LOOP;
    END LOOP;

    ----------------------------------------------------------------------------
    -- 3. PERTE_LIGNE : on tourne vers le dernier côté vu, sans jamais conclure
    --    à une extrémité si la ligne était sur un côté.
    ----------------------------------------------------------------------------
    demi_avant := nb_demi_tours;
    appliquer(CENTRE_GAUCHE, 2);
    appliquer(BLANC, 1);
    verifier_etat(E_PERTE, "perte après centre gauche");
    verifier_mvt(M_PIVOT_GAUCHE, "perte après centre gauche");
    appliquer(BLANC, 3 * FIN_COURT);
    verifier_etat(E_PERTE, "longue perte après centre gauche");
    verifier_mvt(M_PIVOT_GAUCHE, "longue perte après centre gauche");
    appliquer(CENTRE_DROIT, 1);
    verifier_etat(E_SUIVI, "ligne retrouvée par le centre droit");
    verifier_mvt(M_VIRAGE_DROIT, "ligne retrouvée par le centre droit");
    appliquer(BLANC, 2 * FIN_COURT);
    verifier_mvt(M_PIVOT_DROIT, "perte après centre droit");
    appliquer(GAUCHE, 1);
    verifier_mvt(M_PIVOT_GAUCHE, "ligne retrouvée par le capteur gauche");
    appliquer(BLANC, 2 * FIN_COURT);
    verifier_mvt(M_PIVOT_GAUCHE, "perte après gauche");
    appliquer(DROIT, 1);
    appliquer(BLANC, 2 * FIN_COURT);
    verifier_etat(E_PERTE, "perte après droit");
    verifier_mvt(M_PIVOT_DROIT, "perte après droit");
    appliquer("11000", 1);
    appliquer(BLANC, 2);
    verifier_mvt(M_PIVOT_GAUCHE, "perte après gauche et centre gauche");
    appliquer("00011", 1);
    appliquer(BLANC, 2);
    verifier_mvt(M_PIVOT_DROIT, "perte après droit et centre droit");
    verifier(nb_demi_tours = demi_avant, "extrémité comptée après une perte de ligne sur le côté");
    -- Ligne centrée puis perdue : avance tout droit lentement.
    appliquer("01110", 2);
    appliquer(BLANC, 1);
    verifier_etat(E_PERTE, "perte après ligne centrée");
    verifier_mvt(M_AVANCE_LENTE, "perte après ligne centrée");

    ----------------------------------------------------------------------------
    -- 4. Extrémité : exactement FIN_COURT cycles blancs consécutifs.
    ----------------------------------------------------------------------------
    -- Pertes centrées de FIN_COURT - 1 cycles, séparées par un seul cycle
    -- noir : le décompte doit repartir de zéro, aucune extrémité.
    appliquer(CENTRE, 1);
    FOR i IN 1 TO 3 LOOP
      appliquer(BLANC, FIN_COURT - 1);
      verifier_etat(E_PERTE, "perte centrée de FIN - 1 cycles (" & INTEGER'IMAGE(i) & ")");
      verifier_mvt(M_AVANCE_LENTE, "perte centrée de FIN - 1 cycles");
      appliquer(CENTRE, 1);
      verifier_etat(E_SUIVI, "ligne revue après FIN - 1 cycles blancs");
    END LOOP;
    verifier(nb_demi_tours = demi_avant, "extrémité comptée avant FIN_LIGNE_CYCLES_G cycles blancs consécutifs");
    -- Exactement FIN_COURT cycles : demi-tour au front du FIN_COURT-ième.
    appliquer(BLANC, FIN_COURT - 1);
    verifier_etat(E_PERTE, "un cycle avant l'extrémité");
    fronts(1);
    verifier_etat(E_DEMI_TOUR, "extrémité après FIN_LIGNE_CYCLES_G cycles blancs");
    verifier(nb_demi_tours = demi_avant + 1, "un seul demi-tour attendu");

    ----------------------------------------------------------------------------
    -- 5. DEMI_TOUR : pivot à droite jusqu'à ce que le centre voie noir.
    ----------------------------------------------------------------------------
    verifier_mvt(M_PIVOT_DROIT, "demi-tour");
    appliquer(BLANC, 3 * FIN_COURT);
    verifier_etat(E_DEMI_TOUR, "demi-tour, tout blanc");
    FOR v IN 1 TO 31 LOOP
      s := STD_LOGIC_VECTOR(TO_UNSIGNED(v, 5));
      IF s(2) = '0' THEN
        appliquer(s, 1);
        verifier_etat(E_DEMI_TOUR, "demi-tour arrêté sans le centre (" & image(s) & ")");
        verifier_mvt(M_PIVOT_DROIT, "demi-tour (" & image(s) & ")");
      END IF;
    END LOOP;
    appliquer("01110", 1);
    verifier_etat(E_SUIVI, "fin du demi-tour quand le centre voit noir");
    verifier_mvt(M_TOUT_DROIT, "reprise du suivi après le demi-tour");
    -- Le demi-tour avec le centre et un côté reprend aussi le suivi, avec la
    -- correction de ce côté.
    extremite_et_demi_tour("2e extrémité de l'essai");
    appliquer(CENTRE, 2);
    appliquer(BLANC, FIN_COURT);
    verifier_etat(E_DEMI_TOUR, "3e extrémité de l'essai");
    appliquer("01100", 1);
    verifier_etat(E_SUIVI, "fin du demi-tour avec centre et centre gauche");
    verifier_mvt(M_VIRAGE_GAUCHE, "fin du demi-tour avec centre et centre gauche");
    verifier(nb_arrivees = 0, "arrêt avant la 4e extrémité");

    ----------------------------------------------------------------------------
    -- 6. INACTIF remet le compteur d'extrémités (3 ici) et la mémoire du
    --    côté à zéro.
    ----------------------------------------------------------------------------
    appliquer(GAUCHE, 2);
    poser_actif('0');
    fronts(1);
    verifier_etat(E_INACTIF, "désactivation depuis SUIVI");
    verifier_mvt(M_ARRET, "désactivation depuis SUIVI");
    appliquer(BLANC, 3);
    poser_actif('1');
    fronts(1);
    verifier_etat(E_SUIVI, "réactivation");
    fronts(1);
    verifier_etat(E_PERTE, "réactivation sur du blanc");
    verifier_mvt(M_AVANCE_LENTE, "réactivation : mémoire du côté remise à centré");
    appliquer(CENTRE, 2);

    ----------------------------------------------------------------------------
    -- 7. Parcours complet de la piste EMA : 2 allers/retours.
    ----------------------------------------------------------------------------
    demi_avant := nb_demi_tours;
    troncon("aller 1");
    extremite_et_demi_tour("extrémité 1");
    verifier(nb_demi_tours = demi_avant + 1 AND nb_arrivees = 0, "extrémité 1 : un demi-tour attendu");
    troncon("retour 1");
    extremite_et_demi_tour("extrémité 2");
    verifier(nb_demi_tours = demi_avant + 2 AND nb_arrivees = 0, "extrémité 2 : un demi-tour attendu");
    troncon("aller 2");
    extremite_et_demi_tour("extrémité 3");
    verifier(nb_demi_tours = demi_avant + 3 AND nb_arrivees = 0, "pas d'arrêt après 3 extrémités");
    verifier(etat_defaut = E_SUIVI, "NB_FINS_G par défaut : arrêt après 3 extrémités");
    troncon("retour 2");
    appliquer(CENTRE, 3);
    appliquer(BLANC, FIN_COURT - 1);
    verifier_etat(E_PERTE, "arrivée : extrémité trop tôt");
    fronts(1);
    verifier_etat(E_FIN, "arrêt sur le point de départ après 4 extrémités");
    verifier(etat_defaut = E_FIN, "NB_FINS_G par défaut : pas d'arrêt à la 4e extrémité");
    verifier_mvt(M_ARRET, "arrêt sur le point de départ");
    verifier(nb_demi_tours = demi_avant + 3 AND nb_arrivees = 1, "4e extrémité : arrêt au lieu d'un demi-tour");
    -- On y reste, quels que soient les capteurs.
    FOR v IN 0 TO 31 LOOP
      appliquer(STD_LOGIC_VECTOR(TO_UNSIGNED(v, 5)), 2);
      verifier_etat(E_FIN, "fin de parcours quittée (" & INTEGER'IMAGE(v) & ")");
    END LOOP;
    appliquer(BLANC, 5 * FIN_COURT);
    verifier_etat(E_FIN, "fin de parcours quittée sur du blanc");
    verifier_mvt(M_ARRET, "fin de parcours");

    ----------------------------------------------------------------------------
    -- 7 bis. Désactivation depuis FIN_PARCOURS : nouveau parcours complet.
    ----------------------------------------------------------------------------
    poser_actif('0');
    fronts(1);
    verifier_etat(E_INACTIF, "désactivation depuis FIN_PARCOURS");
    appliquer(CENTRE, 2);
    poser_actif('1');
    fronts(2);
    demi_avant := nb_demi_tours;
    FOR i IN 1 TO 3 LOOP
      extremite_et_demi_tour("nouveau parcours, extrémité " & INTEGER'IMAGE(i));
    END LOOP;
    verifier(nb_demi_tours = demi_avant + 3 AND nb_arrivees = 1, "nouveau parcours : compteur non remis à zéro");
    appliquer(CENTRE, 2);
    appliquer(BLANC, FIN_COURT);
    verifier_etat(E_FIN, "nouveau parcours : arrêt à la 4e extrémité");

    ----------------------------------------------------------------------------
    -- 8. Priorités : actif = '0' depuis chaque état, puis batt_vide.
    ----------------------------------------------------------------------------
    -- actif = '0' depuis PERTE_LIGNE et DEMI_TOUR.
    poser_actif('0'); fronts(1);
    appliquer(CENTRE, 1);
    poser_actif('1'); fronts(2);
    appliquer(BLANC, 2);
    verifier_etat(E_PERTE, "préparation désactivation en perte de ligne");
    poser_actif('0'); fronts(1);
    verifier_etat(E_INACTIF, "désactivation depuis PERTE_LIGNE");
    verifier_mvt(M_ARRET, "désactivation depuis PERTE_LIGNE");
    appliquer(CENTRE, 1);
    poser_actif('1'); fronts(2);
    appliquer(BLANC, FIN_COURT);
    verifier_etat(E_DEMI_TOUR, "préparation désactivation en demi-tour");
    poser_actif('0'); fronts(1);
    verifier_etat(E_INACTIF, "désactivation depuis DEMI_TOUR");
    verifier_mvt(M_ARRET, "désactivation depuis DEMI_TOUR");

    -- batt_vide depuis INACTIF (actif = '0') : ARRET_BATTERIE.
    poser_batt('1'); fronts(1);
    verifier_etat(E_BATTERIE, "batterie vide en mode inactif");
    verifier_mvt(M_ARRET, "batterie vide en mode inactif");
    poser_batt('0');
    appliquer(CENTRE, 2);
    poser_actif('1'); fronts(5);
    verifier_etat(E_BATTERIE, "ARRET_BATTERIE quitté sans reset");
    poser_actif('0'); fronts(5);
    verifier_etat(E_BATTERIE, "ARRET_BATTERIE quitté sur actif = '0'");
    -- Reset (horloge figée) : retour en INACTIF ; reset asynchrone immédiat.
    reset_complet;
    verifier_etat(E_INACTIF, "reset depuis ARRET_BATTERIE");

    -- batt_vide depuis SUIVI, avec actif qui retombe au même front : la
    -- batterie est prioritaire.
    poser_actif('1'); fronts(2);
    verifier_etat(E_SUIVI, "préparation batterie vide en suivi");
    WAIT UNTIL FALLING_EDGE(clk);
    batt_vide <= '1';
    actif     <= '0';
    fronts(1);
    verifier_etat(E_BATTERIE, "batt_vide prioritaire sur actif = '0'");
    verifier_mvt(M_ARRET, "batterie vide depuis SUIVI");
    poser_batt('0');
    reset_complet;

    -- batt_vide depuis PERTE_LIGNE, DEMI_TOUR et FIN_PARCOURS.
    appliquer(CENTRE, 1);
    poser_actif('1'); fronts(2);
    appliquer(BLANC, 2);
    poser_batt('1'); fronts(1);
    verifier_etat(E_BATTERIE, "batterie vide depuis PERTE_LIGNE");
    poser_batt('0'); fronts(3);
    verifier_etat(E_BATTERIE, "ARRET_BATTERIE quitté (perte de ligne)");
    reset_complet;
    appliquer(CENTRE, 1);
    poser_actif('1'); fronts(2);
    appliquer(BLANC, FIN_COURT);
    verifier_etat(E_DEMI_TOUR, "préparation batterie vide en demi-tour");
    poser_batt('1'); fronts(1);
    verifier_etat(E_BATTERIE, "batterie vide depuis DEMI_TOUR");
    poser_batt('0');
    reset_complet;
    appliquer(CENTRE, 1);
    poser_actif('1'); fronts(2);
    FOR i IN 1 TO 3 LOOP
      extremite_et_demi_tour("avant batterie vide, extrémité " & INTEGER'IMAGE(i));
    END LOOP;
    appliquer(BLANC, FIN_COURT);
    verifier_etat(E_FIN, "préparation batterie vide en fin de parcours");
    poser_batt('1'); fronts(1);
    verifier_etat(E_BATTERIE, "batterie vide depuis FIN_PARCOURS");
    poser_batt('0');

    -- Reset asynchrone : effet immédiat, sans front d'horloge ; le reset
    -- remet aussi le compteur d'extrémités à zéro.
    reset_complet;
    appliquer(CENTRE, 1);
    poser_actif('1'); fronts(2);
    demi_avant := nb_demi_tours;
    FOR i IN 1 TO 3 LOOP
      extremite_et_demi_tour("après reset, extrémité " & INTEGER'IMAGE(i));
    END LOOP;
    verifier(nb_demi_tours = demi_avant + 3, "reset : compteur d'extrémités non remis à zéro");
    -- Ligne perdue à gauche : le reset doit aussi effacer ce dernier côté vu.
    appliquer(GAUCHE, 1);
    appliquer(BLANC, 3);
    verifier_mvt(M_PIVOT_GAUCHE, "avant reset asynchrone : perte à gauche");
    WAIT FOR 300 ns;
    rst_b <= '0';
    WAIT FOR 1 ns;
    verifier_etat(E_INACTIF, "reset asynchrone sans effet immédiat");
    verifier_mvt(M_ARRET, "reset asynchrone : moteurs non arrêtés");
    WAIT FOR 1 ms;
    rst_b <= '1';
    fronts(1);
    verifier_etat(E_SUIVI, "après reset, mode actif");
    fronts(1);
    verifier_etat(E_PERTE, "après reset, ligne perdue");
    verifier_mvt(M_AVANCE_LENTE, "après reset : mémoire du côté remise à centré");

    ----------------------------------------------------------------------------
    -- 9. Génériques réels : extrémité après 100 000 cycles blancs (200 ms).
    ----------------------------------------------------------------------------
    poser_actif('0');
    WAIT UNTIL FALLING_EDGE(clk);
    actif_reel    <= '1';
    sol_noir_reel <= CENTRE;
    fronts(3);
    verifier(etat_reel = E_SUIVI, "réel : activation");
    WAIT UNTIL FALLING_EDGE(clk);
    sol_noir_reel <= BLANC;
    fronts(FIN_LIGNE_CYCLES - 1);
    verifier(etat_reel = E_PERTE, "réel : extrémité avant 100 000 cycles blancs");
    WAIT UNTIL FALLING_EDGE(clk);
    sol_noir_reel <= CENTRE;
    fronts(1);
    verifier(etat_reel = E_SUIVI, "réel : ligne retrouvée");
    WAIT UNTIL FALLING_EDGE(clk);
    sol_noir_reel <= BLANC;
    fronts(FIN_LIGNE_CYCLES - 1);
    verifier(etat_reel = E_PERTE, "réel : un cycle avant l'extrémité");
    fronts(1);
    verifier(etat_reel = E_DEMI_TOUR, "réel : extrémité non détectée après 100 000 cycles blancs");

    -- Bilan.
    IF nb_echecs = 0 AND erreurs_obs = 0 THEN
      REPORT "TEST OK : tb_suiveur_ligne (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_suiveur_ligne (" & INTEGER'IMAGE(nb_echecs + erreurs_obs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    std.env.finish;
  END PROCESS verification;

END ARCHITECTURE tb_suiveur_ligne_arch;
