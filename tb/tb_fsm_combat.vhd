--------------------------------------------------------------------------------
-- Fichier      : tb_fsm_combat.vhd
-- Entité       : tb_fsm_combat
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant de la machine à états "Combat
--                Sumo" (cahier des charges §6.3) : chaque transition du
--                diagramme, chaque non-transition (l'état ne bouge pas sans
--                condition), les priorités reset > batt_vide > actif = '0' >
--                diagramme, bordure > adversaire, l'irréversibilité de ARRET,
--                les sorties de chaque état (sens et vitesse des roues,
--                ralentissement de la roue du côté de l'adversaire en
--                ATTAQUE seulement : adversaire vu d'un seul côté présenté
--                aussi en POWER_ON, ATTENTE_5S, EVITEMENT et ARRET) et la
--                durée de l'attente, dont la fin ne dépend ni de la bordure
--                ni de l'adversaire.
-- Principe     : Deux exemplaires reçoivent les mêmes entrées :
--                - u_fsm : attente réduite à N = 50 cycles, pour les scénarios ;
--                - u_reel : génériques par défaut, dont on mesure à la fin
--                  l'attente réelle (2 500 000 cycles = 5 s à 500 kHz).
--                L'horloge 500 kHz est figée à '0' pendant chaque reset comme
--                en sortie du diviseur sur la carte : la remise à POWER_ON
--                pendant le reset prouve que le reset est asynchrone.
--                Les bascules internes (côté de l'adversaire, compteur
--                d'attente) sont lues par noms externes pendant chaque reset
--                pour vérifier leur reset asynchrone (règle du cours : reset
--                asynchrone sur toutes les bascules).
--                Les entrées changent 1 ns après un front montant ; un
--                observateur vérifie que les sorties de u_fsm ne changent
--                qu'à un front montant d'horloge ou au reset (sorties de
--                Moore, aucune entrée ne passe directement sur les moteurs).
--                Chien de garde à 8 s simulées. "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY tb_fsm_combat IS
END ENTITY tb_fsm_combat;

ARCHITECTURE tb_fsm_combat_arch OF tb_fsm_combat IS

  CONSTANT PERIODE_CLK : TIME    := 2 us;   -- horloge système 500 kHz
  CONSTANT N           : NATURAL := 50;     -- attente réduite de u_fsm (cycles)

  -- Codes de etat_dbg (identiques à ceux de fsm_combat).
  CONSTANT E_POWER_ON  : STD_LOGIC_VECTOR(2 DOWNTO 0) := "000";
  CONSTANT E_ATTENTE   : STD_LOGIC_VECTOR(2 DOWNTO 0) := "001";
  CONSTANT E_RECHERCHE : STD_LOGIC_VECTOR(2 DOWNTO 0) := "010";
  CONSTANT E_ATTAQUE   : STD_LOGIC_VECTOR(2 DOWNTO 0) := "011";
  CONSTANT E_EVITEMENT : STD_LOGIC_VECTOR(2 DOWNTO 0) := "100";
  CONSTANT E_ARRET     : STD_LOGIC_VECTOR(2 DOWNTO 0) := "101";

  SIGNAL clk        : STD_LOGIC := '0';
  SIGNAL rst_b      : STD_LOGIC := '0';
  SIGNAL fin_simu   : BOOLEAN   := FALSE;

  SIGNAL actif      : STD_LOGIC := '0';
  SIGNAL adversaire : STD_LOGIC := '0';
  SIGNAL adv_gauche : STD_LOGIC := '0';
  SIGNAL adv_droite : STD_LOGIC := '0';
  SIGNAL bordure    : STD_LOGIC := '0';
  SIGNAL batt_vide  : STD_LOGIC := '0';

  SIGNAL dir_gauche, dir_droit   : STD_LOGIC;
  SIGNAL duty_gauche, duty_droit : t_duty;
  SIGNAL etat_dbg                : STD_LOGIC_VECTOR(2 DOWNTO 0);

  SIGNAL dir_gauche_reel, dir_droit_reel   : STD_LOGIC;
  SIGNAL duty_gauche_reel, duty_droit_reel : t_duty;
  SIGNAL etat_dbg_reel                     : STD_LOGIC_VECTOR(2 DOWNTO 0);

  -- Sorties qui changent hors front d'horloge et hors reset.
  SIGNAL echecs_synchrone : NATURAL := 0;

  -- Texte d'un code d'état, pour les messages d'erreur.
  FUNCTION nom_etat (e : STD_LOGIC_VECTOR(2 DOWNTO 0)) RETURN STRING IS
  BEGIN
    CASE e IS
      WHEN "000"  => RETURN "POWER_ON";
      WHEN "001"  => RETURN "ATTENTE_5S";
      WHEN "010"  => RETURN "RECHERCHE_ADVERSAIRE";
      WHEN "011"  => RETURN "ATTAQUE_ADVERSAIRE";
      WHEN "100"  => RETURN "EVITEMENT_BORDURE";
      WHEN "101"  => RETURN "ARRET";
      WHEN OTHERS => RETURN "code invalide";
    END CASE;
  END FUNCTION nom_etat;

BEGIN

  u_fsm : ENTITY WORK.fsm_combat
    GENERIC MAP (ATTENTE_CYCLES_G => N)
    PORT MAP (clk => clk, rst_b => rst_b, actif => actif,
              adversaire => adversaire, adv_gauche => adv_gauche, adv_droite => adv_droite,
              bordure => bordure, batt_vide => batt_vide,
              dir_gauche => dir_gauche, dir_droit => dir_droit,
              duty_gauche => duty_gauche, duty_droit => duty_droit, etat_dbg => etat_dbg);

  u_reel : ENTITY WORK.fsm_combat
    PORT MAP (clk => clk, rst_b => rst_b, actif => actif,
              adversaire => adversaire, adv_gauche => adv_gauche, adv_droite => adv_droite,
              bordure => bordure, batt_vide => batt_vide,
              dir_gauche => dir_gauche_reel, dir_droit => dir_droit_reel,
              duty_gauche => duty_gauche_reel, duty_droit => duty_droit_reel,
              etat_dbg => etat_dbg_reel);

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

  -- Sorties synchrones : toute variation d'une sortie de u_fsm doit avoir lieu
  -- à l'instant d'un front montant de clk ou d'un changement de rst_b. Les
  -- entrées changeant 1 ns après les fronts, une sortie qui dépendrait
  -- directement d'une entrée serait détectée ici.
  observateur_synchrone : PROCESS
  BEGIN
    WAIT ON dir_gauche, dir_droit, duty_gauche, duty_droit, etat_dbg;
    IF NOW > 0 ns THEN
      IF NOT ((clk = '1' AND clk'LAST_EVENT = 0 ns) OR rst_b'LAST_EVENT = 0 ns) THEN
        echecs_synchrone <= echecs_synchrone + 1;
        REPORT "ECHEC : sortie modifiée hors front d'horloge et hors reset (sortie non Moore)"
          SEVERITY ERROR;
      END IF;
    END IF;
  END PROCESS observateur_synchrone;

  -- Chien de garde : le test complet dure environ 5,1 s simulées.
  chien_de_garde : PROCESS
  BEGIN
    WAIT FOR 8 sec;
    IF NOT fin_simu THEN
      REPORT "TEST KO : tb_fsm_combat bloqué (délai de 8 s dépassé)" SEVERITY FAILURE;
    END IF;
    WAIT;
  END PROCESS chien_de_garde;

  verification : PROCESS
    -- Bascules internes de u_fsm (noms externes VHDL-2008) : toutes doivent
    -- avoir un reset asynchrone (règle du cours), même celles dont la valeur
    -- après reset n'est pas visible sur les ports.
    ALIAS reg_adv_seul_gauche IS << SIGNAL .tb_fsm_combat.u_fsm.adv_seul_gauche : STD_LOGIC >>;
    ALIAS reg_adv_seul_droite IS << SIGNAL .tb_fsm_combat.u_fsm.adv_seul_droite : STD_LOGIC >>;
    ALIAS reg_duree_attente   IS << SIGNAL .tb_fsm_combat.u_fsm.duree_attente   : NATURAL RANGE 0 TO N >>;
    VARIABLE nb_controles : NATURAL := 0;
    VARIABLE nb_echecs    : NATURAL := 0;
    VARIABLE nb           : NATURAL;

    PROCEDURE verifier (condition : BOOLEAN; message : STRING) IS
    BEGIN
      nb_controles := nb_controles + 1;
      IF NOT condition THEN
        nb_echecs := nb_echecs + 1;
      END IF;
      ASSERT condition REPORT "ECHEC : " & message SEVERITY ERROR;
    END PROCEDURE verifier;

    -- Attend n fronts montants, puis 1 ns (instant où l'on change les entrées
    -- et où l'on observe les sorties).
    PROCEDURE attendre_cycles (n_cycles : NATURAL) IS
    BEGIN
      FOR i IN 1 TO n_cycles LOOP
        WAIT UNTIL RISING_EDGE(clk);
      END LOOP;
      WAIT FOR 1 ns;
    END PROCEDURE attendre_cycles;

    -- Positionne toutes les entrées de la machine.
    PROCEDURE entrees (a, adv, ag, ad, bord, batt : STD_LOGIC) IS
    BEGIN
      actif <= a; adversaire <= adv; adv_gauche <= ag; adv_droite <= ad;
      bordure <= bord; batt_vide <= batt;
    END PROCEDURE entrees;

    -- Vérifie l'état et toutes les sorties attendues dans cet état.
    -- cote : ' ' attaque droit devant, 'G' adversaire vu seulement à gauche,
    -- 'D' adversaire vu seulement à droite (sans effet hors ATTAQUE).
    PROCEDURE controler (e : STD_LOGIC_VECTOR(2 DOWNTO 0); contexte : STRING;
                         cote : CHARACTER := ' ') IS
      VARIABLE dg, dd : STD_LOGIC;
      VARIABLE vg, vd : NATURAL;
    BEGIN
      verifier(etat_dbg = e, contexte & " : état " & nom_etat(etat_dbg) &
               " au lieu de " & nom_etat(e));
      CASE e IS
        WHEN E_RECHERCHE =>
          -- Rotation sur place : gauche en avant, droite en arrière.
          dg := DIR_AVANT_GAUCHE; dd := NOT DIR_AVANT_DROIT;
          vg := VITESSE_RECHERCHE; vd := VITESSE_RECHERCHE;
        WHEN E_ATTAQUE =>
          dg := DIR_AVANT_GAUCHE; dd := DIR_AVANT_DROIT;
          vg := VITESSE_ATTAQUE; vd := VITESSE_ATTAQUE;
          IF cote = 'G' THEN vg := VITESSE_VIRAGE; END IF;
          IF cote = 'D' THEN vd := VITESSE_VIRAGE; END IF;
        WHEN E_EVITEMENT =>
          dg := NOT DIR_AVANT_GAUCHE; dd := NOT DIR_AVANT_DROIT;
          vg := VITESSE_RECUL; vd := VITESSE_RECUL;
        WHEN OTHERS =>
          -- POWER_ON, ATTENTE_5S, ARRET : moteurs coupés.
          dg := DIR_AVANT_GAUCHE; dd := DIR_AVANT_DROIT;
          vg := 0; vd := 0;
      END CASE;
      verifier(duty_gauche = TO_UNSIGNED(vg, t_duty'LENGTH),
               contexte & " : duty_gauche = " & INTEGER'IMAGE(TO_INTEGER(duty_gauche)) &
               " au lieu de " & INTEGER'IMAGE(vg));
      verifier(duty_droit = TO_UNSIGNED(vd, t_duty'LENGTH),
               contexte & " : duty_droit = " & INTEGER'IMAGE(TO_INTEGER(duty_droit)) &
               " au lieu de " & INTEGER'IMAGE(vd));
      verifier(dir_gauche = dg, contexte & " : sens roue gauche incorrect");
      verifier(dir_droit = dd, contexte & " : sens roue droite incorrect");
    END PROCEDURE controler;

    -- L'état reste e pendant n cycles avec les entrées courantes.
    PROCEDURE rester (e : STD_LOGIC_VECTOR(2 DOWNTO 0); n_cycles : NATURAL;
                      contexte : STRING; cote : CHARACTER := ' ') IS
      VARIABLE ecarts : NATURAL := 0;
    BEGIN
      FOR i IN 1 TO n_cycles LOOP
        attendre_cycles(1);
        IF etat_dbg /= e THEN ecarts := ecarts + 1; END IF;
      END LOOP;
      verifier(ecarts = 0, contexte & " : l'état a quitté " & nom_etat(e) & " " &
               INTEGER'IMAGE(ecarts) & " fois sur " & INTEGER'IMAGE(n_cycles) & " cycles");
      controler(e, contexte, cote);
    END PROCEDURE rester;

    -- Reset de 20 µs, horloge figée : la remise à POWER_ON doit être immédiate
    -- (asynchrone). Les entrées sont remises au repos avant le relâchement.
    PROCEDURE faire_reset (contexte : STRING) IS
    BEGIN
      rst_b <= '0';
      WAIT FOR 1 ns;
      controler(E_POWER_ON, contexte & " (reset asynchrone, horloge figée)");
      verifier(reg_adv_seul_gauche = '0' AND reg_adv_seul_droite = '0',
               contexte & " : bascules de côté adversaire non remises à 0 par le reset");
      verifier(reg_duree_attente = 0,
               contexte & " : compteur d'attente non remis à 0 par le reset");
      entrees('0', '0', '0', '0', '0', '0');
      WAIT FOR 20 us;
      rst_b <= '1';
      attendre_cycles(1);
      controler(E_POWER_ON, contexte & " (premier front après reset)");
    END PROCEDURE faire_reset;

    -- Depuis un reset, amène la machine en RECHERCHE_ADVERSAIRE.
    PROCEDURE aller_recherche (contexte : STRING) IS
    BEGIN
      faire_reset(contexte);
      actif <= '1';
      attendre_cycles(1);
      controler(E_ATTENTE, contexte & " : entrée en attente");
      attendre_cycles(N);
      controler(E_RECHERCHE, contexte & " : fin de l'attente");
    END PROCEDURE aller_recherche;

    -- Depuis un reset, amène la machine en ATTAQUE_ADVERSAIRE (droit devant).
    PROCEDURE aller_attaque (contexte : STRING) IS
    BEGIN
      aller_recherche(contexte);
      entrees('1', '1', '1', '1', '0', '0');
      attendre_cycles(1);
      controler(E_ATTAQUE, contexte & " : entrée en attaque");
    END PROCEDURE aller_attaque;

    -- Depuis un reset, amène la machine en EVITEMENT_BORDURE.
    PROCEDURE aller_evitement (contexte : STRING) IS
    BEGIN
      aller_recherche(contexte);
      entrees('1', '0', '0', '0', '1', '0');
      attendre_cycles(1);
      controler(E_EVITEMENT, contexte & " : entrée en évitement");
    END PROCEDURE aller_evitement;

    -- Compte les fronts montants pendant lesquels etat_dbg vaut e (sortie de
    -- l'exemplaire choisi), au plus limite.
    PROCEDURE compter_cycles_dans (SIGNAL s : IN STD_LOGIC_VECTOR(2 DOWNTO 0);
                                   e : STD_LOGIC_VECTOR(2 DOWNTO 0);
                                   limite : NATURAL; resultat : OUT NATURAL) IS
      VARIABLE c : NATURAL := 0;
    BEGIN
      WHILE s = e AND c <= limite LOOP
        c := c + 1;
        attendre_cycles(1);
      END LOOP;
      resultat := c;
    END PROCEDURE compter_cycles_dans;

  BEGIN
    ----------------------------------------------------------------------------
    -- 0. Valeur réelle de l'attente : 5 s à 500 kHz.
    ----------------------------------------------------------------------------
    verifier(ATTENTE_COMBAT_CYCLES = 5 * CLK_SYS_HZ,
             "ATTENTE_COMBAT_CYCLES ne vaut pas 5 s à 500 kHz");
    verifier(ATTENTE_COMBAT_CYCLES = 2_500_000, "ATTENTE_COMBAT_CYCLES /= 2 500 000");

    ----------------------------------------------------------------------------
    -- 1. Mise sous tension : POWER_ON pendant le reset, quelles que soient
    --    les entrées ; puis POWER_ON tant que le mode combat n'est pas actif.
    ----------------------------------------------------------------------------
    entrees('1', '1', '1', '0', '1', '0');
    WAIT FOR 10 us;
    controler(E_POWER_ON, "pendant le reset de mise sous tension");
    entrees('0', '0', '0', '0', '0', '0');
    WAIT FOR 10 us;
    rst_b <= '1';
    attendre_cycles(1);
    controler(E_POWER_ON, "premier front après mise sous tension");
    rester(E_POWER_ON, 10, "POWER_ON, mode inactif");
    entrees('0', '1', '1', '1', '1', '0');
    rester(E_POWER_ON, 10, "POWER_ON, mode inactif, adversaire et bordure");
    -- Adversaire vu d'un seul côté : aucune roue ne doit tourner hors ATTAQUE.
    entrees('0', '1', '1', '0', '1', '0');
    rester(E_POWER_ON, 5, "POWER_ON, mode inactif, adversaire à gauche seulement");
    entrees('0', '1', '0', '1', '1', '0');
    rester(E_POWER_ON, 5, "POWER_ON, mode inactif, adversaire à droite seulement");

    ----------------------------------------------------------------------------
    -- 2. POWER_ON -> ATTENTE_5S dès que actif = '1' ; durée exacte de N cycles,
    --    ni l'adversaire ni la bordure ne l'écourtent ni ne la prolongent, et
    --    l'attente mène toujours à la recherche (le '1' du diagramme).
    ----------------------------------------------------------------------------
    entrees('1', '0', '0', '0', '0', '0');
    WAIT FOR 1 ns;
    controler(E_POWER_ON, "actif vient de passer à 1, avant le front");
    attendre_cycles(1);
    controler(E_ATTENTE, "POWER_ON + actif");
    -- Adversaire vu d'un seul côté puis de l'autre : moteurs toujours coupés.
    entrees('1', '1', '1', '0', '1', '0');      -- sans effet pendant l'attente
    rester(E_ATTENTE, N / 4, "attente, bordure et adversaire à gauche seulement ignorés");
    entrees('1', '1', '0', '1', '1', '0');
    rester(E_ATTENTE, N / 2 - 1 - N / 4,
           "attente, bordure et adversaire à droite seulement ignorés");
    -- Bordure et adversaire maintenus jusqu'à la fin de l'attente.
    entrees('1', '1', '1', '1', '1', '0');
    compter_cycles_dans(etat_dbg, E_ATTENTE, 3 * N, nb);
    verifier(nb = N / 2 + 1, "durée de l'attente avec bordure et adversaire : " &
             INTEGER'IMAGE(N / 2 - 1 + nb) & " cycles au lieu de " & INTEGER'IMAGE(N));
    controler(E_RECHERCHE, "fin de l'attente, bordure et adversaire vus");
    attendre_cycles(1);
    controler(E_EVITEMENT, "recherche juste après l'attente + bordure (bordure prioritaire)");
    entrees('1', '0', '0', '0', '0', '0');
    attendre_cycles(1);
    controler(E_RECHERCHE, "évitement après l'attente + fin de bordure");

    -- Second passage : adversaire vu à gauche seulement pendant toute
    -- l'attente, qui dure N cycles et finit en recherche, puis attaque.
    faire_reset("préparation fin d'attente avec adversaire");
    entrees('1', '1', '1', '0', '0', '0');
    attendre_cycles(1);
    controler(E_ATTENTE, "POWER_ON + actif, adversaire à gauche seulement");
    compter_cycles_dans(etat_dbg, E_ATTENTE, 3 * N, nb);
    verifier(nb = N, "durée de l'attente avec adversaire : " & INTEGER'IMAGE(nb) &
             " cycles au lieu de " & INTEGER'IMAGE(N));
    controler(E_RECHERCHE, "fin de l'attente, adversaire vu à gauche seulement");
    attendre_cycles(1);
    controler(E_ATTAQUE, "recherche juste après l'attente + adversaire à gauche", 'G');
    entrees('1', '0', '0', '0', '0', '0');
    attendre_cycles(1);
    controler(E_RECHERCHE, "attaque après l'attente + adversaire perdu");

    ----------------------------------------------------------------------------
    -- 3. RECHERCHE_ADVERSAIRE : non-transition, puis vers EVITEMENT (bordure,
    --    prioritaire sur l'adversaire) et vers ATTAQUE (adversaire seul).
    ----------------------------------------------------------------------------
    entrees('1', '0', '0', '0', '0', '0');
    rester(E_RECHERCHE, 20, "recherche sans adversaire ni bordure");
    entrees('1', '1', '1', '1', '1', '0');
    attendre_cycles(1);
    controler(E_EVITEMENT, "recherche + adversaire + bordure (bordure prioritaire)");

    ----------------------------------------------------------------------------
    -- 4. EVITEMENT_BORDURE : reste tant que la bordure est vue (même avec un
    --    adversaire), puis repasse TOUJOURS par la recherche.
    ----------------------------------------------------------------------------
    rester(E_EVITEMENT, 20, "évitement, bordure et adversaire vus");
    -- Adversaire vu d'un seul côté : recul symétrique, aucune roue ralentie.
    entrees('1', '1', '1', '0', '1', '0');
    rester(E_EVITEMENT, 5, "évitement, bordure et adversaire à gauche seulement");
    entrees('1', '1', '0', '1', '1', '0');
    rester(E_EVITEMENT, 5, "évitement, bordure et adversaire à droite seulement");
    entrees('1', '0', '0', '0', '1', '0');
    rester(E_EVITEMENT, 5, "évitement, bordure seule");
    entrees('1', '1', '1', '1', '0', '0');
    attendre_cycles(1);
    controler(E_RECHERCHE, "évitement + fin de bordure (adversaire vu) : recherche d'abord");
    attendre_cycles(1);
    controler(E_ATTAQUE, "recherche + adversaire");

    ----------------------------------------------------------------------------
    -- 5. ATTAQUE_ADVERSAIRE : sorties selon le côté où l'adversaire est vu
    --    (prises en compte au front suivant), non-transition.
    ----------------------------------------------------------------------------
    rester(E_ATTAQUE, 20, "attaque, adversaire vu des deux côtés");
    entrees('1', '1', '1', '0', '0', '0');
    WAIT FOR 1 ns;
    controler(E_ATTAQUE, "attaque, côté gauche seul avant le front");
    attendre_cycles(1);
    controler(E_ATTAQUE, "attaque, adversaire à gauche seulement", 'G');
    rester(E_ATTAQUE, 5, "attaque, adversaire à gauche seulement", 'G');
    entrees('1', '1', '0', '1', '0', '0');
    attendre_cycles(1);
    controler(E_ATTAQUE, "attaque, adversaire à droite seulement", 'D');
    rester(E_ATTAQUE, 5, "attaque, adversaire à droite seulement", 'D');
    entrees('1', '1', '1', '1', '0', '0');
    attendre_cycles(1);
    controler(E_ATTAQUE, "attaque, adversaire revu des deux côtés");
    entrees('1', '1', '0', '0', '0', '0');
    rester(E_ATTAQUE, 3, "attaque, adversaire sans côté identifié");

    ----------------------------------------------------------------------------
    -- 6. ATTAQUE -> RECHERCHE (adversaire perdu), RECHERCHE -> ATTAQUE avec
    --    un seul capteur (ralentissement dès le premier cycle d'attaque),
    --    ATTAQUE -> EVITEMENT (bordure, avec ou sans adversaire).
    ----------------------------------------------------------------------------
    entrees('1', '0', '0', '0', '0', '0');
    attendre_cycles(1);
    controler(E_RECHERCHE, "attaque + adversaire perdu");
    entrees('1', '1', '0', '1', '0', '0');
    attendre_cycles(1);
    controler(E_ATTAQUE, "recherche + adversaire à droite seulement", 'D');
    entrees('1', '1', '1', '1', '1', '0');
    attendre_cycles(1);
    controler(E_EVITEMENT, "attaque + bordure + adversaire");
    entrees('1', '0', '0', '0', '0', '0');
    attendre_cycles(1);
    controler(E_RECHERCHE, "évitement + fin de bordure");
    entrees('1', '1', '0', '0', '0', '0');
    attendre_cycles(1);
    controler(E_ATTAQUE, "recherche + adversaire (signal global seul)");
    entrees('1', '0', '0', '0', '1', '0');
    attendre_cycles(1);
    controler(E_EVITEMENT, "attaque + bordure, adversaire perdu (bordure prioritaire)");
    entrees('1', '0', '0', '0', '0', '0');
    attendre_cycles(1);
    controler(E_RECHERCHE, "évitement + fin de bordure, sans adversaire");

    ----------------------------------------------------------------------------
    -- 7. Reset asynchrone depuis chaque état actif (horloge figée).
    ----------------------------------------------------------------------------
    faire_reset("reset depuis RECHERCHE");
    aller_attaque("préparation reset en ATTAQUE");
    faire_reset("reset depuis ATTAQUE");
    aller_attaque("préparation reset en ATTAQUE côté gauche");
    entrees('1', '1', '1', '0', '0', '0');
    attendre_cycles(1);
    controler(E_ATTAQUE, "préparation reset en ATTAQUE côté gauche", 'G');
    faire_reset("reset depuis ATTAQUE côté gauche");
    aller_attaque("préparation reset en ATTAQUE côté droit");
    entrees('1', '1', '0', '1', '0', '0');
    attendre_cycles(1);
    controler(E_ATTAQUE, "préparation reset en ATTAQUE côté droit", 'D');
    faire_reset("reset depuis ATTAQUE côté droit");
    aller_evitement("préparation reset en EVITEMENT");
    faire_reset("reset depuis EVITEMENT");
    actif <= '1';
    attendre_cycles(3);
    controler(E_ATTENTE, "préparation reset en ATTENTE");
    faire_reset("reset depuis ATTENTE");

    ----------------------------------------------------------------------------
    -- 8. actif = '0' : retour à POWER_ON depuis chaque état, prioritaire sur
    --    la bordure, l'adversaire et la fin de l'attente. L'attente repart
    --    ensuite de zéro (durée complète de N cycles).
    ----------------------------------------------------------------------------
    actif <= '1';
    attendre_cycles(1);
    controler(E_ATTENTE, "entrée en attente avant abandon");
    attendre_cycles(N - 1);                      -- dernier cycle de l'attente
    controler(E_ATTENTE, "dernier cycle de l'attente");
    actif <= '0';
    attendre_cycles(1);
    controler(E_POWER_ON, "attente + actif = 0 au dernier cycle");
    actif <= '1';
    attendre_cycles(1);
    controler(E_ATTENTE, "retour en attente après abandon");
    compter_cycles_dans(etat_dbg, E_ATTENTE, 3 * N, nb);
    verifier(nb = N, "attente relancée : " & INTEGER'IMAGE(nb) & " cycles au lieu de " &
             INTEGER'IMAGE(N) & " (compteur non remis à zéro)");
    controler(E_RECHERCHE, "fin de l'attente relancée");

    entrees('0', '1', '1', '1', '1', '0');
    attendre_cycles(1);
    controler(E_POWER_ON, "recherche + actif = 0 (prioritaire sur bordure)");
    aller_attaque("préparation abandon en ATTAQUE");
    entrees('0', '1', '1', '1', '1', '0');
    attendre_cycles(1);
    controler(E_POWER_ON, "attaque + actif = 0 (prioritaire sur bordure)");
    aller_evitement("préparation abandon en EVITEMENT");
    entrees('0', '1', '1', '1', '0', '0');
    attendre_cycles(1);
    controler(E_POWER_ON, "évitement + actif = 0 (prioritaire sur fin de bordure)");
    rester(E_POWER_ON, 5, "POWER_ON après abandon");

    ----------------------------------------------------------------------------
    -- 9. batt_vide : ARRET depuis chaque état, prioritaire sur actif = '0',
    --    sur la bordure, l'adversaire et la fin de l'attente.
    ----------------------------------------------------------------------------
    entrees('0', '0', '0', '0', '0', '1');
    attendre_cycles(1);
    controler(E_ARRET, "POWER_ON (mode inactif) + batt_vide");
    faire_reset("sortie d'ARRET par reset");

    actif <= '1';
    attendre_cycles(N);                          -- dernier cycle de l'attente
    controler(E_ATTENTE, "attente avant batt_vide");
    entrees('1', '1', '1', '1', '1', '1');
    attendre_cycles(1);
    controler(E_ARRET, "attente (dernier cycle) + batt_vide");

    aller_recherche("préparation batt_vide en RECHERCHE");
    entrees('0', '1', '1', '1', '1', '1');
    attendre_cycles(1);
    controler(E_ARRET, "recherche + batt_vide + actif = 0 (batt_vide prioritaire)");

    aller_attaque("préparation batt_vide en ATTAQUE");
    entrees('1', '1', '1', '0', '1', '1');
    attendre_cycles(1);
    controler(E_ARRET, "attaque + batt_vide + bordure");

    aller_evitement("préparation batt_vide en EVITEMENT");
    entrees('1', '1', '1', '1', '0', '1');
    attendre_cycles(1);
    controler(E_ARRET, "évitement + batt_vide + fin de bordure");

    ----------------------------------------------------------------------------
    -- 10. ARRET est définitif : aucune entrée ne l'en fait sortir.
    ----------------------------------------------------------------------------
    entrees('1', '0', '0', '0', '0', '0');
    rester(E_ARRET, 10, "ARRET, batterie revenue, mode actif");
    entrees('0', '0', '0', '0', '0', '0');
    rester(E_ARRET, 10, "ARRET, mode inactif");
    entrees('1', '0', '0', '0', '0', '0');
    rester(E_ARRET, 10, "ARRET, mode réactivé");
    entrees('1', '1', '1', '0', '0', '0');
    rester(E_ARRET, 10, "ARRET, adversaire vu à gauche seulement");
    entrees('1', '1', '0', '1', '0', '0');
    rester(E_ARRET, 10, "ARRET, adversaire vu à droite seulement");
    entrees('1', '0', '0', '0', '1', '0');
    rester(E_ARRET, 10, "ARRET, bordure vue");
    entrees('1', '1', '1', '1', '1', '1');
    rester(E_ARRET, 10, "ARRET, toutes les entrées à 1");

    -- Reset avec batterie toujours vide : retour immédiat en ARRET.
    rst_b <= '0';
    WAIT FOR 1 ns;
    controler(E_POWER_ON, "reset depuis ARRET (asynchrone)");
    WAIT FOR 20 us;
    entrees('1', '0', '0', '0', '0', '1');
    rst_b <= '1';
    attendre_cycles(1);
    controler(E_ARRET, "après reset, batterie toujours vide");
    faire_reset("reset final depuis ARRET");
    rester(E_POWER_ON, 5, "POWER_ON après reset, batterie revenue");

    ----------------------------------------------------------------------------
    -- 11. Attente réelle (générique par défaut) : exactement 2 500 000 cycles
    --     (5 s à 500 kHz) entre POWER_ON et RECHERCHE_ADVERSAIRE.
    ----------------------------------------------------------------------------
    faire_reset("préparation attente réelle");
    verifier(etat_dbg_reel = E_POWER_ON, "attente réelle : u_reel pas en POWER_ON après reset");
    actif <= '1';
    attendre_cycles(1);
    verifier(etat_dbg_reel = E_ATTENTE, "attente réelle : u_reel pas en ATTENTE_5S");
    verifier(duty_gauche_reel = 0 AND duty_droit_reel = 0, "attente réelle : moteurs non arrêtés");
    compter_cycles_dans(etat_dbg_reel, E_ATTENTE, ATTENTE_COMBAT_CYCLES + 10, nb);
    verifier(nb = ATTENTE_COMBAT_CYCLES, "attente réelle de " & INTEGER'IMAGE(nb) &
             " cycles au lieu de " & INTEGER'IMAGE(ATTENTE_COMBAT_CYCLES));
    verifier(etat_dbg_reel = E_RECHERCHE, "attente réelle : u_reel pas en recherche après 5 s");
    verifier(duty_gauche_reel = VITESSE_RECHERCHE AND duty_droit_reel = VITESSE_RECHERCHE,
             "attente réelle : u_reel ne tourne pas après 5 s");

    -- Bilan.
    IF nb_echecs = 0 AND echecs_synchrone = 0 THEN
      REPORT "TEST OK : tb_fsm_combat (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_fsm_combat (" & INTEGER'IMAGE(nb_echecs + echecs_synchrone) &
             " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    std.env.finish;
  END PROCESS verification;

END ARCHITECTURE tb_fsm_combat_arch;
