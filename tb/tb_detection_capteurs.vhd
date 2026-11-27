--------------------------------------------------------------------------------
-- Fichier      : tb_detection_capteurs.vhd
-- Entité       : tb_detection_capteurs
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant de la mise en forme des
--                capteurs : couleur du sol, bordure et son maintien,
--                capteurs IR d'obstacle, code batterie.
-- Principe     : Horloge 500 kHz figée à '0' pendant les resets, comme en
--                sortie du diviseur de fréquence sur la carte. Deux
--                exemplaires reçoivent les mêmes entrées :
--                - u_court : maintien réduit à 20 cycles et seuil à 1000
--                  (contrôle des génériques, tests rapides) ;
--                - u_reel : valeurs par défaut (seuil 2048, maintien
--                  150 000 cycles = 300 ms), mesurées exactement.
--                Les entrées changent 1 ns après un front montant ; on
--                vérifie les sorties 1 ns après chaque front suivant.
--                Contrôles : valeurs pendant et juste après un reset (de
--                mise sous tension et asynchrone en fonctionnement) ;
--                latence de recopie des voies (sol_noir 2 fronts, code
--                batterie 1 front) ; seuil de chaque voie au code près et
--                correspondance bit / voie (y compris voies 0, 6, 7 sans
--                effet) ; durée exacte du maintien de bordure et relance par
--                un clair d'un seul cycle ; latence (2 bascules) et polarité
--                des IR, sans inversion gauche / droite.
--                Surveillance permanente des deux exemplaires : bordure à
--                '1' dès qu'un bit de sol_noir est à '0', adversaire =
--                adv_gauche OR adv_droite, et contrôle de structure (chaque
--                sortie égale sa bascule interne, lue par nom externe).
--                Chien de garde à 1 s. "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY tb_detection_capteurs IS
END ENTITY tb_detection_capteurs;

ARCHITECTURE tb_detection_capteurs_arch OF tb_detection_capteurs IS

  CONSTANT PERIODE_CLK  : TIME    := 2 us;    -- horloge système 500 kHz
  CONSTANT M_COURT      : NATURAL := 20;      -- maintien de bordure de u_court
  CONSTANT SEUIL_COURT  : NATURAL := 1000;    -- seuil noir de u_court
  CONSTANT CODE_NOIR    : NATURAL := 3000;    -- noir pour les deux exemplaires
  CONSTANT CODE_CLAIR   : NATURAL := 0;       -- clair pour les deux exemplaires

  SIGNAL clk         : STD_LOGIC := '0';
  SIGNAL rst_b       : STD_LOGIC := '0';
  SIGNAL fin_simu    : BOOLEAN   := FALSE;

  -- Entrées communes aux deux exemplaires.
  SIGNAL voies       : t_voies_adc := (OTHERS => (OTHERS => '0'));
  SIGNAL ir_gauche_b : STD_LOGIC := '1';
  SIGNAL ir_droit_b  : STD_LOGIC := '1';

  -- Sorties de u_court (suffixe _c) et de u_reel (suffixe _r).
  SIGNAL sol_noir_c, sol_noir_r : STD_LOGIC_VECTOR(4 DOWNTO 0);
  SIGNAL bordure_c, bordure_r   : STD_LOGIC;
  SIGNAL adv_g_c, adv_g_r       : STD_LOGIC;
  SIGNAL adv_d_c, adv_d_r       : STD_LOGIC;
  SIGNAL adv_c, adv_r           : STD_LOGIC;
  SIGNAL code_c, code_r         : t_mot_adc;

  -- Compteurs d'échecs des processus de surveillance (un pilote chacun).
  SIGNAL echecs_invariants : NATURAL := 0;
  SIGNAL echecs_structure  : NATURAL := 0;

  -- Voies toutes au même code.
  FUNCTION toutes (code : NATURAL) RETURN t_voies_adc IS
  BEGIN
    RETURN (OTHERS => TO_UNSIGNED(code, 12));
  END FUNCTION toutes;

  -- sol_noir attendu pour des voies données et un seuil donné (bit 4 =
  -- gauche, 3 = centre gauche, 2 = centre, 1 = centre droit, 0 = droit).
  FUNCTION sol_attendu (v : t_voies_adc; seuil : NATURAL) RETURN STD_LOGIC_VECTOR IS
    TYPE t_correspondance IS ARRAY (4 DOWNTO 0) OF NATURAL;
    CONSTANT VOIE_DU_BIT : t_correspondance :=
      (4 => VOIE_SOL_GAUCHE, 3 => VOIE_SOL_CENTRE_GAUCHE, 2 => VOIE_SOL_CENTRE,
       1 => VOIE_SOL_CENTRE_DROIT, 0 => VOIE_SOL_DROIT);
    VARIABLE r : STD_LOGIC_VECTOR(4 DOWNTO 0);
  BEGIN
    FOR b IN 4 DOWNTO 0 LOOP
      IF TO_INTEGER(v(VOIE_DU_BIT(b))) > seuil THEN r(b) := '1'; ELSE r(b) := '0'; END IF;
    END LOOP;
    RETURN r;
  END FUNCTION sol_attendu;

  FUNCTION image (s : STD_LOGIC_VECTOR) RETURN STRING IS
    VARIABLE r : STRING(1 TO s'LENGTH);
    VARIABLE i : POSITIVE := 1;
  BEGIN
    FOR b IN s'RANGE LOOP
      r(i) := STD_LOGIC'IMAGE(s(b))(2);
      i := i + 1;
    END LOOP;
    RETURN r;
  END FUNCTION image;

BEGIN

  u_court : ENTITY WORK.detection_capteurs
    GENERIC MAP (BORDURE_MAINTIEN_CYCLES_G => M_COURT, SEUIL_NOIR_G => SEUIL_COURT)
    PORT MAP (clk => clk, rst_b => rst_b, voies => voies,
              ir_gauche_b => ir_gauche_b, ir_droit_b => ir_droit_b,
              sol_noir => sol_noir_c, bordure => bordure_c,
              adv_gauche => adv_g_c, adv_droite => adv_d_c, adversaire => adv_c,
              code_batterie => code_c);

  u_reel : ENTITY WORK.detection_capteurs
    PORT MAP (clk => clk, rst_b => rst_b, voies => voies,
              ir_gauche_b => ir_gauche_b, ir_droit_b => ir_droit_b,
              sol_noir => sol_noir_r, bordure => bordure_r,
              adv_gauche => adv_g_r, adv_droite => adv_d_r, adversaire => adv_r,
              code_batterie => code_r);

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

  -- Invariants vérifiés à chaque front, pour les deux exemplaires :
  -- bordure à '1' dès qu'un capteur voit clair (au même cycle que sol_noir),
  -- et adversaire = adv_gauche OR adv_droite.
  invariants : PROCESS
  BEGIN
    WAIT UNTIL RISING_EDGE(clk);
    WAIT FOR 1 ns;
    IF (sol_noir_c /= "11111" AND bordure_c /= '1') OR
       (sol_noir_r /= "11111" AND bordure_r /= '1') THEN
      echecs_invariants <= echecs_invariants + 1;
      REPORT "ECHEC : un capteur voit clair mais bordure n'est pas à '1'" SEVERITY ERROR;
    END IF;
    IF adv_c /= (adv_g_c OR adv_d_c) OR adv_r /= (adv_g_r OR adv_d_r) THEN
      echecs_invariants <= echecs_invariants + 1;
      REPORT "ECHEC : adversaire différent de adv_gauche OR adv_droite" SEVERITY ERROR;
    END IF;
  END PROCESS invariants;

  -- Contrôle de structure : chaque sortie est à tout instant sa bascule
  -- interne (lue par nom externe), vérifié 1 ns après chaque front et chaque
  -- changement du reset.
  sorties_registrees : PROCESS
    ALIAS sol_c  IS << SIGNAL .tb_detection_capteurs.u_court.sol_noir_reg   : STD_LOGIC_VECTOR(4 DOWNTO 0) >>;
    ALIAS bor_c  IS << SIGNAL .tb_detection_capteurs.u_court.bordure_reg    : STD_LOGIC >>;
    ALIAS ag_c   IS << SIGNAL .tb_detection_capteurs.u_court.adv_gauche_reg : STD_LOGIC >>;
    ALIAS ad_c   IS << SIGNAL .tb_detection_capteurs.u_court.adv_droite_reg : STD_LOGIC >>;
    ALIAS a_c    IS << SIGNAL .tb_detection_capteurs.u_court.adversaire_reg : STD_LOGIC >>;
    ALIAS v_c    IS << SIGNAL .tb_detection_capteurs.u_court.voies_reg      : t_voies_adc >>;
    ALIAS sol_r  IS << SIGNAL .tb_detection_capteurs.u_reel.sol_noir_reg    : STD_LOGIC_VECTOR(4 DOWNTO 0) >>;
    ALIAS bor_r  IS << SIGNAL .tb_detection_capteurs.u_reel.bordure_reg     : STD_LOGIC >>;
    ALIAS ag_r   IS << SIGNAL .tb_detection_capteurs.u_reel.adv_gauche_reg  : STD_LOGIC >>;
    ALIAS ad_r   IS << SIGNAL .tb_detection_capteurs.u_reel.adv_droite_reg  : STD_LOGIC >>;
    ALIAS a_r    IS << SIGNAL .tb_detection_capteurs.u_reel.adversaire_reg  : STD_LOGIC >>;
    ALIAS v_r    IS << SIGNAL .tb_detection_capteurs.u_reel.voies_reg       : t_voies_adc >>;
  BEGIN
    WAIT ON clk, rst_b;
    WAIT FOR 1 ns;
    IF sol_noir_c /= sol_c OR bordure_c /= bor_c OR adv_g_c /= ag_c OR adv_d_c /= ad_c OR
       adv_c /= a_c OR code_c /= v_c(VOIE_BAT) OR
       sol_noir_r /= sol_r OR bordure_r /= bor_r OR adv_g_r /= ag_r OR adv_d_r /= ad_r OR
       adv_r /= a_r OR code_r /= v_r(VOIE_BAT) THEN
      echecs_structure <= echecs_structure + 1;
      REPORT "ECHEC : une sortie n'est pas la sortie directe de sa bascule" SEVERITY ERROR;
    END IF;
  END PROCESS sorties_registrees;

  -- Chien de garde : le test dure environ 310 ms (maintien réel de 300 ms).
  chien_de_garde : PROCESS
  BEGIN
    WAIT FOR 1000 ms;
    IF NOT fin_simu THEN
      REPORT "TEST KO : tb_detection_capteurs bloqué (délai de 1 s dépassé)" SEVERITY FAILURE;
    END IF;
    WAIT;
  END PROCESS chien_de_garde;

  verification : PROCESS
    VARIABLE nb_controles : NATURAL := 0;
    VARIABLE nb_echecs    : NATURAL := 0;

    PROCEDURE verifier (condition : BOOLEAN; message : STRING) IS
    BEGIN
      nb_controles := nb_controles + 1;
      IF NOT condition THEN
        nb_echecs := nb_echecs + 1;
      END IF;
      ASSERT condition REPORT "ECHEC : " & message SEVERITY ERROR;
    END PROCEDURE verifier;

    -- Attend n fronts montants, puis 1 ns (sorties stabilisées).
    PROCEDURE cycles (n : NATURAL) IS
    BEGIN
      FOR i IN 1 TO n LOOP
        WAIT UNTIL RISING_EDGE(clk);
      END LOOP;
      WAIT FOR 1 ns;
    END PROCEDURE cycles;

    -- Valeurs de reset des sorties des deux exemplaires.
    PROCEDURE verifier_reset (contexte : STRING) IS
    BEGIN
      verifier(sol_noir_c = "00000" AND sol_noir_r = "00000",
               contexte & " : sol_noir vaut " & image(sol_noir_c) & " / " & image(sol_noir_r) & " au lieu de 00000");
      verifier(bordure_c = '1' AND bordure_r = '1', contexte & " : bordure devrait être à '1'");
      verifier(adv_g_c = '0' AND adv_g_r = '0' AND adv_d_c = '0' AND adv_d_r = '0' AND
               adv_c = '0' AND adv_r = '0', contexte & " : adversaire signalé pendant le reset");
      verifier(code_c = 0 AND code_r = 0, contexte & " : code_batterie non nul pendant le reset");
    END PROCEDURE verifier_reset;

    -- Les voies v étant appliquées depuis au moins 2 fronts : sol_noir des
    -- deux exemplaires conforme à leur seuil.
    PROCEDURE verifier_sol (v : t_voies_adc; contexte : STRING) IS
    BEGIN
      verifier(sol_noir_c = sol_attendu(v, SEUIL_COURT),
               contexte & " : sol_noir (seuil 1000) = " & image(sol_noir_c) &
               " au lieu de " & image(sol_attendu(v, SEUIL_COURT)));
      verifier(sol_noir_r = sol_attendu(v, SEUIL_NOIR),
               contexte & " : sol_noir (seuil 2048) = " & image(sol_noir_r) &
               " au lieu de " & image(sol_attendu(v, SEUIL_NOIR)));
    END PROCEDURE verifier_sol;

    -- Appelée 1 ns après le front où sol_noir de l'exemplaire est passé à
    -- "11111" : compte les cycles où bordure reste à '1' (ce cycle compris)
    -- tant que le sol reste noir, jusqu'à limite cycles au plus.
    PROCEDURE mesurer_maintien (SIGNAL bor : IN STD_LOGIC; SIGNAL sol : IN STD_LOGIC_VECTOR;
                                limite : NATURAL; n : OUT NATURAL) IS
      VARIABLE compte : NATURAL := 0;
    BEGIN
      WHILE bor = '1' AND sol = "11111" AND compte < limite LOOP
        compte := compte + 1;
        cycles(1);
      END LOOP;
      n := compte;
    END PROCEDURE mesurer_maintien;

    VARIABLE v     : t_voies_adc;
    VARIABLE n     : NATURAL;
    VARIABLE ok    : BOOLEAN;
    TYPE t_codes IS ARRAY (NATURAL RANGE <>) OF NATURAL;
    CONSTANT CODES_TESTES : t_codes := (0, SEUIL_COURT, SEUIL_COURT + 1, SEUIL_NOIR,
                                        SEUIL_NOIR + 1, 4095);
    CONSTANT CODES_BAT    : t_codes := (0, BAT_CODE_MIN, BAT_CODE_MAX, 4095, 1);

  BEGIN
    ----------------------------------------------------------------------------
    -- 0. Valeurs réelles des génériques par défaut.
    ----------------------------------------------------------------------------
    verifier(BORDURE_MAINTIEN_CYCLES = 300 * (CLK_SYS_HZ / 1000),
             "BORDURE_MAINTIEN_CYCLES ne vaut pas 300 ms à 500 kHz");
    verifier(SEUIL_NOIR = 2048, "SEUIL_NOIR de départ différent de 2048 (mi-échelle)");

    ----------------------------------------------------------------------------
    -- 1. Mise sous tension : reset de 1 ms, horloge figée. Les entrées
    --    « actives » ne doivent pas apparaître en sortie.
    ----------------------------------------------------------------------------
    voies       <= toutes(CODE_NOIR);
    ir_gauche_b <= '0';
    ir_droit_b  <= '0';
    WAIT FOR 100 us;
    verifier_reset("mise sous tension");
    WAIT FOR 900 us;
    verifier_reset("fin du reset de mise sous tension");
    -- Capteurs IR au repos avant le relâchement : aucun adversaire, pas même
    -- pendant le premier cycle (valeur de reset des bascules de synchro).
    ir_gauche_b <= '1';
    ir_droit_b  <= '1';
    voies       <= toutes(CODE_CLAIR);
    WAIT FOR 1 us;
    rst_b <= '1';
    FOR i IN 1 TO 4 LOOP
      cycles(1);
      verifier(adv_g_c = '0' AND adv_d_c = '0' AND adv_c = '0' AND
               adv_g_r = '0' AND adv_d_r = '0' AND adv_r = '0',
               "adversaire signalé au cycle " & INTEGER'IMAGE(i) & " après le reset (capteurs au repos)");
    END LOOP;

    ----------------------------------------------------------------------------
    -- 2. Latence de recopie : sol_noir change au 2e front (registre 500 kHz
    --    puis bascule de sortie), code_batterie au 1er front.
    ----------------------------------------------------------------------------
    v := toutes(CODE_NOIR);
    v(VOIE_BAT) := TO_UNSIGNED(1234, 12);
    voies <= v;
    WAIT FOR 1 ns;
    verifier(code_c = 0 AND code_r = 0, "code_batterie suit l'entrée sans attendre un front");
    cycles(1);
    verifier(code_c = 1234 AND code_r = 1234, "code_batterie pas à jour après 1 front");
    verifier(sol_noir_c = "00000" AND sol_noir_r = "00000",
             "sol_noir à jour dès le 1er front : les voies ne sont pas recopiées avant le calcul");
    cycles(1);
    verifier(sol_noir_c = "11111" AND sol_noir_r = "11111", "sol_noir pas à jour après 2 fronts");
    verifier(bordure_c = '1', "bordure tombée au moment même où le sol devient noir");

    ----------------------------------------------------------------------------
    -- 3. Seuils et correspondance bit / voie : chaque voie, une à la fois, à
    --    différents codes, les autres toutes noires puis toutes claires.
    --    Les voies 0, 6 et 7 ne doivent jamais modifier sol_noir.
    ----------------------------------------------------------------------------
    FOR k IN 0 TO 7 LOOP
      FOR c IN CODES_TESTES'RANGE LOOP
        -- Autres voies noires : seul le bit de la voie k peut être à '0'.
        v := toutes(CODE_NOIR);
        v(k) := TO_UNSIGNED(CODES_TESTES(c), 12);
        voies <= v;
        cycles(2);
        verifier_sol(v, "voie " & INTEGER'IMAGE(k) & " à " & INTEGER'IMAGE(CODES_TESTES(c)) & ", autres noires");
        -- Autres voies claires : seul le bit de la voie k peut être à '1'.
        v := toutes(CODE_CLAIR);
        v(k) := TO_UNSIGNED(CODES_TESTES(c), 12);
        voies <= v;
        cycles(2);
        verifier_sol(v, "voie " & INTEGER'IMAGE(k) & " à " & INTEGER'IMAGE(CODES_TESTES(c)) & ", autres claires");
        verifier(bordure_c = '1' AND bordure_r = '1', "bordure absente alors que des capteurs voient clair");
      END LOOP;
    END LOOP;

    ----------------------------------------------------------------------------
    -- 4. Maintien exact de bordure (u_court, 20 cycles) puis absence de
    --    bordure tant que tout reste noir.
    ----------------------------------------------------------------------------
    voies <= toutes(CODE_NOIR);
    cycles(2);
    verifier(sol_noir_c = "11111", "maintien : sol pas noir");
    mesurer_maintien(bordure_c, sol_noir_c, M_COURT + 10, n);
    verifier(n = M_COURT, "maintien de bordure de " & INTEGER'IMAGE(n) & " cycles au lieu de " &
             INTEGER'IMAGE(M_COURT));
    ok := TRUE;
    FOR i IN 1 TO 2 * M_COURT LOOP
      IF bordure_c /= '0' THEN ok := FALSE; END IF;
      cycles(1);
    END LOOP;
    verifier(ok, "bordure réapparue alors que le sol reste noir");

    -- Les voies batterie et non câblées (0, 6, 7) au clair ne créent pas de
    -- bordure.
    v := toutes(CODE_NOIR);
    v(VOIE_BAT) := TO_UNSIGNED(CODE_CLAIR, 12);
    v(6) := TO_UNSIGNED(CODE_CLAIR, 12);
    v(7) := TO_UNSIGNED(CODE_CLAIR, 12);
    voies <= v;
    ok := TRUE;
    FOR i IN 1 TO 5 LOOP
      cycles(1);
      IF bordure_c /= '0' OR sol_noir_c /= "11111" THEN ok := FALSE; END IF;
    END LOOP;
    verifier(ok, "les voies 0, 6 ou 7 au clair déclenchent la bordure");

    -- Chaque capteur de sol seul au clair pendant 1 cycle déclenche la
    -- bordure, puis un maintien complet.
    FOR k IN 1 TO 5 LOOP
      v := toutes(CODE_NOIR);
      v(k) := TO_UNSIGNED(CODE_CLAIR, 12);
      voies <= v;
      cycles(1);
      voies <= toutes(CODE_NOIR);
      cycles(1);
      verifier(bordure_c = '1', "clair d'un cycle sur la voie " & INTEGER'IMAGE(k) & " : pas de bordure");
      cycles(1);
      mesurer_maintien(bordure_c, sol_noir_c, M_COURT + 10, n);
      verifier(n = M_COURT, "voie " & INTEGER'IMAGE(k) & " : maintien de " & INTEGER'IMAGE(n) &
               " cycles au lieu de " & INTEGER'IMAGE(M_COURT));
      cycles(2);
    END LOOP;

    ----------------------------------------------------------------------------
    -- 5. Relance du maintien : un clair d'un seul cycle au milieu du maintien
    --    le fait repartir du début.
    ----------------------------------------------------------------------------
    v := toutes(CODE_NOIR);
    v(VOIE_SOL_DROIT) := TO_UNSIGNED(CODE_CLAIR, 12);
    voies <= v;
    cycles(1);
    voies <= toutes(CODE_NOIR);
    cycles(2);                              -- sol_noir redevient "11111"
    mesurer_maintien(bordure_c, sol_noir_c, M_COURT / 2, n);  -- moitié du maintien
    v := toutes(CODE_NOIR);
    v(VOIE_SOL_GAUCHE) := TO_UNSIGNED(CODE_CLAIR, 12);
    voies <= v;
    cycles(1);
    voies <= toutes(CODE_NOIR);
    cycles(1);
    verifier(sol_noir_c = "01111" AND bordure_c = '1', "relance : clair d'un cycle mal vu");
    cycles(1);
    mesurer_maintien(bordure_c, sol_noir_c, 2 * M_COURT, n);
    verifier(n = M_COURT, "maintien relancé de " & INTEGER'IMAGE(n) & " cycles au lieu de " &
             INTEGER'IMAGE(M_COURT));

    ----------------------------------------------------------------------------
    -- 6. Capteurs IR : 2 bascules (sortie au 2e front), polarité inversée,
    --    gauche et droite non permutés.
    ----------------------------------------------------------------------------
    -- Obstacle à gauche.
    ir_gauche_b <= '0';
    cycles(1);
    verifier(adv_g_c = '0' AND adv_g_r = '0', "IR gauche : sortie dès le 1er front (une seule bascule)");
    cycles(1);
    verifier(adv_g_c = '1' AND adv_g_r = '1', "IR gauche : obstacle non signalé après 2 fronts");
    verifier(adv_d_c = '0' AND adv_d_r = '0', "IR gauche : obstacle signalé à droite");
    verifier(adv_c = '1' AND adv_r = '1', "IR gauche : adversaire non signalé");
    ir_gauche_b <= '1';
    cycles(1);
    verifier(adv_g_c = '1' AND adv_c = '1', "IR gauche : fin d'obstacle dès le 1er front");
    cycles(1);
    verifier(adv_g_c = '0' AND adv_g_r = '0' AND adv_c = '0' AND adv_r = '0',
             "IR gauche : fin d'obstacle non prise en compte après 2 fronts");
    -- Obstacle à droite.
    ir_droit_b <= '0';
    cycles(1);
    verifier(adv_d_c = '0' AND adv_d_r = '0' AND adv_c = '0', "IR droit : sortie dès le 1er front (une seule bascule)");
    cycles(1);
    verifier(adv_d_c = '1' AND adv_d_r = '1', "IR droit : obstacle non signalé après 2 fronts");
    verifier(adv_g_c = '0' AND adv_g_r = '0', "IR droit : obstacle signalé à gauche");
    verifier(adv_c = '1' AND adv_r = '1', "IR droit : adversaire non signalé");
    -- Obstacle des deux côtés, puis fin à droite seulement.
    ir_gauche_b <= '0';
    cycles(2);
    verifier(adv_g_c = '1' AND adv_d_c = '1' AND adv_c = '1', "IR deux côtés : mal signalé");
    ir_droit_b <= '1';
    cycles(2);
    verifier(adv_g_c = '1' AND adv_d_c = '0' AND adv_c = '1', "IR gauche seul après fin à droite : mal signalé");
    ir_gauche_b <= '1';
    cycles(2);
    verifier(adv_g_c = '0' AND adv_d_c = '0' AND adv_c = '0', "IR : adversaire encore signalé sans obstacle");

    ----------------------------------------------------------------------------
    -- 7. code_batterie : voie VOIE_BAT seule, mise à jour en 1 front, sans
    --    dépendre des autres voies.
    ----------------------------------------------------------------------------
    FOR c IN CODES_BAT'RANGE LOOP
      FOR k IN 0 TO 7 LOOP
        v(k) := TO_UNSIGNED(100 * k + 7, 12);
      END LOOP;
      v(VOIE_BAT) := TO_UNSIGNED(CODES_BAT(c), 12);
      voies <= v;
      cycles(1);
      verifier(code_c = CODES_BAT(c) AND code_r = CODES_BAT(c),
               "code_batterie = " & INTEGER'IMAGE(TO_INTEGER(code_c)) & " au lieu de " & INTEGER'IMAGE(CODES_BAT(c)));
    END LOOP;
    FOR k IN 1 TO 7 LOOP
      v(k) := TO_UNSIGNED(4095 - k, 12);
    END LOOP;
    voies <= v;
    cycles(2);
    verifier(code_c = CODES_BAT(CODES_BAT'HIGH), "code_batterie modifié par une autre voie");

    ----------------------------------------------------------------------------
    -- 8. Reset asynchrone en fonctionnement (sol noir, bordure retombée,
    --    obstacle des deux côtés) : rst_b tombe pendant la demi-période basse
    --    de l'horloge, donc sans aucun événement sur clk (comme sur la carte
    --    quand rst_b tombe pendant que clk_500KHz est bas). Seule une liste de
    --    sensibilité contenant rst_b remet alors les sorties à leurs valeurs
    --    de reset. Puis redémarrage propre.
    ----------------------------------------------------------------------------
    v := toutes(CODE_NOIR);
    v(VOIE_BAT) := TO_UNSIGNED(3000, 12);
    voies <= v;
    ir_gauche_b <= '0';
    ir_droit_b  <= '0';
    cycles(2 * M_COURT + 5);
    verifier(bordure_c = '0' AND sol_noir_c = "11111" AND adv_c = '1' AND code_c = 3000,
             "état avant reset asynchrone non atteint");
    -- cycles() rend la main 1 ns après le front montant : 1501 ns après ce
    -- front, clk est à '0' depuis 500 ns et le restera encore 499 ns.
    WAIT FOR 1500 ns;                       -- au milieu de la demi-période basse
    verifier(clk = '0', "reset asynchrone : horloge non basse à l'assertion du reset");
    rst_b <= '0';
    WAIT FOR 1 ns;
    verifier_reset("reset asynchrone");
    WAIT FOR 20 us;
    verifier_reset("pendant le reset asynchrone");
    rst_b <= '1';
    -- 1er front : la synchro IR sort encore de sa valeur de repos.
    cycles(1);
    verifier(adv_g_c = '0' AND adv_d_c = '0' AND adv_c = '0',
             "après reset : obstacle signalé dès le 1er front");
    verifier(sol_noir_c = "00000" AND bordure_c = '1', "après reset : sol / bordure au 1er front");
    cycles(1);
    verifier(adv_g_c = '1' AND adv_d_c = '1' AND adv_c = '1', "après reset : obstacle non signalé au 2e front");
    verifier(sol_noir_c = "11111", "après reset : sol noir non vu au 2e front");
    mesurer_maintien(bordure_c, sol_noir_c, M_COURT + 10, n);
    verifier(n = M_COURT, "après reset : maintien de " & INTEGER'IMAGE(n) & " cycles au lieu de " &
             INTEGER'IMAGE(M_COURT));

    ----------------------------------------------------------------------------
    -- 9. Maintien réel (u_reel, valeurs par défaut) : 150 000 cycles exacts
    --    après un clair d'un cycle.
    ----------------------------------------------------------------------------
    v := toutes(CODE_NOIR);
    v(VOIE_SOL_CENTRE) := TO_UNSIGNED(CODE_CLAIR, 12);
    voies <= v;
    cycles(1);
    voies <= toutes(CODE_NOIR);
    cycles(2);
    verifier(sol_noir_r = "11111" AND bordure_r = '1', "maintien réel : départ incorrect");
    mesurer_maintien(bordure_r, sol_noir_r, BORDURE_MAINTIEN_CYCLES + 10, n);
    verifier(n = BORDURE_MAINTIEN_CYCLES, "maintien réel de " & INTEGER'IMAGE(n) & " cycles au lieu de " &
             INTEGER'IMAGE(BORDURE_MAINTIEN_CYCLES) & " (300 ms)");
    cycles(3);
    verifier(bordure_r = '0', "maintien réel : bordure réapparue");

    -- Bilan (contrôles directs + surveillance permanente).
    WAIT FOR 1 ns;
    IF nb_echecs = 0 AND echecs_invariants = 0 AND echecs_structure = 0 THEN
      REPORT "TEST OK : tb_detection_capteurs (" & INTEGER'IMAGE(nb_controles) &
             " controles + surveillance permanente)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_detection_capteurs (" &
             INTEGER'IMAGE(nb_echecs + echecs_invariants + echecs_structure) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    std.env.finish;
  END PROCESS verification;

END ARCHITECTURE tb_detection_capteurs_arch;
