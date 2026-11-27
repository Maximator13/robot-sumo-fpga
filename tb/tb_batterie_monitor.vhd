--------------------------------------------------------------------------------
-- Fichier      : tb_batterie_monitor.vhd
-- Entité       : tb_batterie_monitor
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant de la surveillance batterie
--                (cahier des charges §6.4) : loi de luminosité de LED7,
--                filtre de batterie vide, mémorisation, clignotement 1 Hz.
-- Principe     : Horloge 500 kHz figée à '0' pendant chaque reset, comme en
--                sortie du diviseur de fréquence sur la carte. Trois
--                exemplaires du module :
--                - u_batterie_monitor : génériques réduits (filtre de 1000
--                  cycles, demi-période de clignotement de 30 cycles), PWM
--                  réelle de 100 cycles ;
--                - u_pwm_200 : même code d'entrée, PWM de 200 cycles, pour
--                  vérifier que le générique PERIODE_PWM_G est transmis ;
--                - u_reel : génériques par défaut (valeurs de la carte),
--                  filtre de 1 s et clignotement 1 Hz mesurés en temps.
--                Le rapport cyclique de LED7 est mesuré en comptant les
--                cycles à '1' sur une période complète, une fois la consigne
--                prise en compte. Contrôles : 9 V -> 100 %, 7,2 V -> 10 %,
--                chaque code de 7,2 V à 9 V égal à la formule et à moins
--                d'un point de la droite idéale, saturation au-dessus de
--                9 V, 10 % sous 7,2 V pendant le filtrage, code nul après
--                reset sans fausse batterie vide, batt_vide après exactement
--                FILTRE_BATT_CYCLES_G cycles consécutifs (bornes du seuil
--                incluses), chutes brèves ignorées, mémorisation jusqu'au
--                reset, clignotement 50 % de période 2 x CLIGNO_DEMI_CYCLES_G
--                commençant allumé, reset asynchrone (y compris en plein
--                clignotement et en pleine impulsion PWM : clignotement et
--                PWM repartent de zéro), sorties registrées
--                (noms externes). Chien de garde à 5 s.
--                "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE IEEE.MATH_REAL.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY tb_batterie_monitor IS
END ENTITY tb_batterie_monitor;

ARCHITECTURE tb_batterie_monitor_arch OF tb_batterie_monitor IS

  CONSTANT PERIODE_CLK : TIME    := 2 us;   -- horloge système 500 kHz
  CONSTANT P           : NATURAL := PWM_PERIODE_CYCLES;   -- 100 cycles
  CONSTANT FILTRE_TB   : NATURAL := 1000;   -- filtre réduit (> 5 périodes PWM de mesure)
  CONSTANT CLIGNO_TB   : NATURAL := 30;     -- demi-période de clignotement réduite
  CONSTANT P_200       : NATURAL := 200;    -- période PWM de l'exemplaire u_pwm_200

  SIGNAL clk      : STD_LOGIC := '0';
  SIGNAL rst_b    : STD_LOGIC := '0';
  SIGNAL fin_simu : BOOLEAN   := FALSE;

  -- Exemplaires à génériques réduits (même code batterie).
  SIGNAL code        : t_mot_adc := (OTHERS => '0');
  SIGNAL led7        : STD_LOGIC;
  SIGNAL batt_vide   : STD_LOGIC;
  SIGNAL led7_200    : STD_LOGIC;
  SIGNAL batt_vide_200 : STD_LOGIC;

  -- Exemplaire réel.
  SIGNAL code_reel      : t_mot_adc := TO_UNSIGNED(BAT_CODE_MAX, 12);
  SIGNAL led7_reel      : STD_LOGIC;
  SIGNAL batt_vide_reel : STD_LOGIC;

  SIGNAL echecs_structure : NATURAL := 0;

  -- Rapport cyclique attendu (formule sans diviseur, CDC §6.4 et paquetage sumo_pkg).
  FUNCTION duty_formule (c : NATURAL) RETURN NATURAL IS
  BEGIN
    IF c >= BAT_CODE_MAX THEN
      RETURN 100;
    ELSIF c < BAT_CODE_MIN THEN
      RETURN 10;
    ELSE
      RETURN 10 + ((c - BAT_CODE_MIN) * PENTE_LED7_NUM) / 2 ** PENTE_LED7_DECALAGE;
    END IF;
  END FUNCTION duty_formule;

  -- Rapport cyclique idéal du CDC : droite de 10 % (7,2 V) à 100 % (9 V).
  FUNCTION duty_ideal (c : NATURAL) RETURN REAL IS
  BEGIN
    IF c >= BAT_CODE_MAX THEN
      RETURN 100.0;
    ELSIF c < BAT_CODE_MIN THEN
      RETURN 10.0;
    ELSE
      RETURN 10.0 + 90.0 * REAL(c - BAT_CODE_MIN) / REAL(BAT_CODE_MAX - BAT_CODE_MIN);
    END IF;
  END FUNCTION duty_ideal;

  -- Code ADC d'une tension de piles : (Vbat - 0,7) / 3, sur 4096 pas de 3,3 V.
  FUNCTION code_de_tension (vbat : REAL) RETURN NATURAL IS
  BEGIN
    RETURN NATURAL(ROUND((vbat - 0.7) / 3.0 * 4096.0 / 3.3));
  END FUNCTION code_de_tension;

BEGIN

  u_batterie_monitor : ENTITY WORK.batterie_monitor
    GENERIC MAP (CLIGNO_DEMI_CYCLES_G => CLIGNO_TB, FILTRE_BATT_CYCLES_G => FILTRE_TB,
                 PERIODE_PWM_G => P)
    PORT MAP (clk => clk, rst_b => rst_b, code_batterie => code,
              led7 => led7, batt_vide => batt_vide);

  u_pwm_200 : ENTITY WORK.batterie_monitor
    GENERIC MAP (CLIGNO_DEMI_CYCLES_G => CLIGNO_TB, FILTRE_BATT_CYCLES_G => FILTRE_TB,
                 PERIODE_PWM_G => P_200)
    PORT MAP (clk => clk, rst_b => rst_b, code_batterie => code,
              led7 => led7_200, batt_vide => batt_vide_200);

  u_reel : ENTITY WORK.batterie_monitor
    PORT MAP (clk => clk, rst_b => rst_b, code_batterie => code_reel,
              led7 => led7_reel, batt_vide => batt_vide_reel);

  -- Horloge 500 kHz comme en sortie du diviseur de fréquence : '0' pendant le
  -- reset (y compris si le reset arrive pendant la demi-période haute),
  -- premier front montant une demi-période après le relâchement.
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

  -- Contrôle de structure : led7 et batt_vide sont à tout instant les
  -- bascules internes led7_reg et batt_vide_reg (vérifié 1 ns après chaque
  -- front d'horloge et chaque changement du reset).
  sorties_registrees : PROCESS
    ALIAS led7_r  IS << SIGNAL .tb_batterie_monitor.u_batterie_monitor.led7_reg      : STD_LOGIC >>;
    ALIAS vide_r  IS << SIGNAL .tb_batterie_monitor.u_batterie_monitor.batt_vide_reg : STD_LOGIC >>;
    ALIAS led7_rr IS << SIGNAL .tb_batterie_monitor.u_reel.led7_reg                  : STD_LOGIC >>;
    ALIAS vide_rr IS << SIGNAL .tb_batterie_monitor.u_reel.batt_vide_reg             : STD_LOGIC >>;
  BEGIN
    WAIT ON clk, rst_b;
    WAIT FOR 1 ns;
    IF led7 /= led7_r OR batt_vide /= vide_r OR led7_reel /= led7_rr OR batt_vide_reel /= vide_rr THEN
      echecs_structure <= echecs_structure + 1;
      REPORT "ECHEC : led7 / batt_vide ne sont pas les sorties directes de leurs bascules" SEVERITY ERROR;
    END IF;
  END PROCESS sorties_registrees;

  -- Chien de garde : le test complet dure environ 2,5 s simulées.
  chien_de_garde : PROCESS
  BEGIN
    WAIT FOR 5 sec;
    IF NOT fin_simu THEN
      REPORT "TEST KO : tb_batterie_monitor bloqué (délai de 5 s dépassé)" SEVERITY FAILURE;
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

    -- Attend n fronts montants d'horloge (échantillonnage 1 ns après).
    PROCEDURE attendre_cycles (n : NATURAL) IS
    BEGIN
      FOR i IN 1 TO n LOOP
        WAIT UNTIL RISING_EDGE(clk);
      END LOOP;
      WAIT FOR 1 ns;
    END PROCEDURE attendre_cycles;

    -- Nombre de cycles à '1' d'un signal sur n cycles consécutifs.
    PROCEDURE compter_hauts (SIGNAL s : IN STD_LOGIC; n : NATURAL; hauts : OUT NATURAL) IS
      VARIABLE h : NATURAL := 0;
    BEGIN
      FOR i IN 1 TO n LOOP
        attendre_cycles(1);
        IF s = '1' THEN
          h := h + 1;
        END IF;
      END LOOP;
      hauts := h;
    END PROCEDURE compter_hauts;

    -- Applique un code, attend la prise en compte par la PWM (consigne
    -- registrée puis lue en fin de période, sortie registrée), puis mesure
    -- le rapport cyclique de LED7 sur une période complète. Avec une période
    -- de 100 cycles, le nombre de cycles à '1' est le rapport cyclique en %.
    -- Durée : 3 périodes, inférieure au filtre réduit.
    PROCEDURE mesurer_duty (c : NATURAL; duty : OUT NATURAL) IS
    BEGIN
      code <= TO_UNSIGNED(c, 12);
      attendre_cycles(2 * P + 2);
      compter_hauts(led7, P, duty);
    END PROCEDURE mesurer_duty;

    -- Mesure et contrôle d'un code : formule exacte, et écart à la droite
    -- idéale du CDC inférieur ou égal à 1 point.
    PROCEDURE tester_code (c : NATURAL; contexte : STRING) IS
      VARIABLE d : NATURAL;
    BEGIN
      mesurer_duty(c, d);
      verifier(d = duty_formule(c), contexte & " : code " & INTEGER'IMAGE(c) & " -> " &
               INTEGER'IMAGE(d) & " % au lieu de " & INTEGER'IMAGE(duty_formule(c)) & " %");
      verifier(ABS(REAL(d) - duty_ideal(c)) <= 1.0, contexte & " : code " & INTEGER'IMAGE(c) &
               " -> " & INTEGER'IMAGE(d) & " %, à plus d'un point de la droite idéale");
      verifier(batt_vide = '0', contexte & " : batt_vide inattendu (code " & INTEGER'IMAGE(c) & ")");
    END PROCEDURE tester_code;

    -- Garde le code c pendant n fronts, en vérifiant après chacun que
    -- batt_vide reste à '0'.
    PROCEDURE tenir_sans_vide (c : NATURAL; n : NATURAL; contexte : STRING) IS
      VARIABLE vu_vide : BOOLEAN := FALSE;
    BEGIN
      code <= TO_UNSIGNED(c, 12);
      FOR i IN 1 TO n LOOP
        attendre_cycles(1);
        IF batt_vide /= '0' THEN vu_vide := TRUE; END IF;
      END LOOP;
      verifier(NOT vu_vide, contexte & " : batt_vide à '1' trop tôt");
    END PROCEDURE tenir_sans_vide;

    -- Consigne LED7 registrée : sans effet visible si elle n'est pas remise à
    -- zéro (réécrite au premier front, lue par la PWM en fin de période),
    -- d'où un contrôle par nom externe de son reset asynchrone.
    ALIAS duty_led_r IS << SIGNAL .tb_batterie_monitor.u_batterie_monitor.duty_led : t_duty >>;

    -- Reset asynchrone de durée d (horloge figée par le processus horloge).
    PROCEDURE faire_reset (d : TIME) IS
    BEGIN
      rst_b <= '0';
      WAIT FOR 1 ns;
      verifier(led7 = '0' AND batt_vide = '0' AND led7_reel = '0' AND batt_vide_reel = '0',
               "le reset asynchrone ne remet pas immédiatement led7 et batt_vide à '0'");
      verifier(duty_led_r = 0, "le reset asynchrone ne remet pas la consigne LED7 à zéro");
      WAIT FOR d - 1 ns;
      verifier(led7 = '0' AND batt_vide = '0', "led7 ou batt_vide non nul pendant le reset");
      rst_b <= '1';
    END PROCEDURE faire_reset;

    -- Appelée au front où batt_vide vient de passer à '1' : on remet un
    -- code de 9 V (la PWM serait à '1' en permanence), puis batt_vide doit
    -- rester à '1' et LED7 alterner exactement CLIGNO_TB cycles à '1' puis
    -- CLIGNO_TB à '0', en commençant par la phase allumée au cycle qui suit
    -- la déclaration (compteur et phase du clignotement bien initialisés).
    PROCEDURE verifier_clignotement (contexte : STRING) IS
      VARIABLE erreurs : NATURAL := 0;
      VARIABLE attendu : STD_LOGIC;
    BEGIN
      code <= TO_UNSIGNED(BAT_CODE_MAX, 12);
      FOR i IN 1 TO 8 * CLIGNO_TB LOOP
        attendre_cycles(1);
        IF ((i - 1) / CLIGNO_TB) MOD 2 = 0 THEN attendu := '1'; ELSE attendu := '0'; END IF;
        IF led7 /= attendu OR batt_vide /= '1' THEN
          erreurs := erreurs + 1;
        END IF;
      END LOOP;
      verifier(erreurs = 0, contexte & " : clignotement incorrect ou batt_vide retombé (" &
               INTEGER'IMAGE(erreurs) & " cycles faux sur " & INTEGER'IMAGE(8 * CLIGNO_TB) & ")");
    END PROCEDURE verifier_clignotement;

    VARIABLE d, h       : NATURAL;
    VARIABLE t_rel, t1, t2, t3 : TIME;
    VARIABLE fronts     : NATURAL;

  BEGIN
    ----------------------------------------------------------------------------
    -- 0. Valeurs réelles : codes ADC de 9 V et 7,2 V, filtre de 1 s,
    --    clignotement à 1 Hz (sur l'horloge 500 kHz).
    ----------------------------------------------------------------------------
    verifier(BAT_CODE_MAX = code_de_tension(9.0), "BAT_CODE_MAX ne correspond pas à 9 V");
    verifier(BAT_CODE_MIN = code_de_tension(7.2), "BAT_CODE_MIN ne correspond pas à 7,2 V");
    verifier(FILTRE_BATT_CYCLES = CLK_SYS_HZ, "FILTRE_BATT_CYCLES différent de 1 s");
    verifier(2 * CLIGNO_DEMI_CYCLES = CLK_SYS_HZ, "clignotement différent de 1 Hz");

    ----------------------------------------------------------------------------
    -- 1. Mise sous tension : reset de 20 µs (horloge figée), sorties à '0'.
    ----------------------------------------------------------------------------
    WAIT FOR 20 us;
    verifier(led7 = '0' AND batt_vide = '0', "sorties non nulles pendant le reset de mise sous tension");
    verifier(led7_reel = '0' AND batt_vide_reel = '0', "u_reel : sorties non nulles pendant le reset");
    rst_b <= '1';

    ----------------------------------------------------------------------------
    -- 2. Juste après le reset, le code vaut 0 (première mesure ADC pas encore
    --    faite) : LED7 à 10 %, pas de fausse batterie vide.
    ----------------------------------------------------------------------------
    tester_code(0, "code nul après reset");

    ----------------------------------------------------------------------------
    -- 3. Loi LED7 : 9 V -> 100 %, 7,2 V -> 10 %, milieu ~55 %, puis chaque
    --    code de la plage (formule exacte et écart <= 1 point).
    ----------------------------------------------------------------------------
    tester_code(BAT_CODE_MAX, "9 V");
    mesurer_duty(BAT_CODE_MAX, d);
    verifier(d = 100, "9 V : LED7 à " & INTEGER'IMAGE(d) & " % au lieu de 100 %");
    mesurer_duty(BAT_CODE_MIN, d);
    verifier(d = 10, "7,2 V : LED7 à " & INTEGER'IMAGE(d) & " % au lieu de 10 %");
    mesurer_duty((BAT_CODE_MIN + BAT_CODE_MAX) / 2, d);
    verifier(d >= 54 AND d <= 56, "8,1 V : LED7 à " & INTEGER'IMAGE(d) & " % au lieu de ~55 %");
    FOR c IN BAT_CODE_MIN TO BAT_CODE_MAX LOOP
      tester_code(c, "plage 7,2 V - 9 V");
    END LOOP;

    ----------------------------------------------------------------------------
    -- 4. Saturation au-dessus de 9 V, et 10 % sous 7,2 V tant que le filtre
    --    n'a pas conclu (chaque mesure basse est suivie d'un code correct).
    ----------------------------------------------------------------------------
    tester_code(BAT_CODE_MAX + 1, "au-dessus de 9 V");
    tester_code(3800, "au-dessus de 9 V");
    tester_code(4095, "pleine échelle");
    tester_code(BAT_CODE_MIN - 1, "sous 7,2 V pendant le filtrage");
    tester_code(3000, "retour normal");
    tester_code(1000, "très bas pendant le filtrage");
    tester_code(3000, "retour normal");

    -- Générique PERIODE_PWM_G transmis : à 9 V, u_pwm_200 donne 100 cycles
    -- à '1' sur 200, en une seule impulsion par période de 200 cycles.
    code <= TO_UNSIGNED(BAT_CODE_MAX, 12);
    attendre_cycles(2 * P_200 + 2);
    WAIT UNTIL RISING_EDGE(led7_200) FOR 2 * P_200 * PERIODE_CLK;
    t1 := NOW;
    WAIT UNTIL FALLING_EDGE(led7_200) FOR 2 * P_200 * PERIODE_CLK;
    t2 := NOW;
    WAIT UNTIL RISING_EDGE(led7_200) FOR 2 * P_200 * PERIODE_CLK;
    t3 := NOW;
    verifier(t3 - t1 = P_200 * PERIODE_CLK, "PERIODE_PWM_G = 200 : période LED7 de " & TIME'IMAGE(t3 - t1));
    verifier(t2 - t1 = 100 * PERIODE_CLK, "PERIODE_PWM_G = 200 : impulsion de " & TIME'IMAGE(t2 - t1));

    ----------------------------------------------------------------------------
    -- 5. Filtre : décompte de cycles CONSÉCUTIFS, bornes du seuil.
    --    a) FILTRE - 1 cycles à BAT_CODE_MIN - 1, un cycle à BAT_CODE_MIN
    --       (qui n'est pas "sous le seuil"), puis encore FILTRE - 1 : rien.
    --    b) chutes brèves répétées (démarrage moteurs) : rien.
    --    c) exactement FILTRE cycles sous le seuil : batt_vide au FILTRE-ième
    --       front, pas avant.
    ----------------------------------------------------------------------------
    tenir_sans_vide(3000, 5, "préparation");
    tenir_sans_vide(BAT_CODE_MIN - 1, FILTRE_TB - 1, "filtre a (1)");
    tenir_sans_vide(BAT_CODE_MIN, 1, "filtre a (seuil)");
    tenir_sans_vide(BAT_CODE_MIN - 1, FILTRE_TB - 1, "filtre a (2)");
    tenir_sans_vide(BAT_CODE_MAX, 1, "filtre a (remontée)");
    FOR i IN 1 TO 10 LOOP
      tenir_sans_vide(0, FILTRE_TB / 2, "chutes brèves");
      tenir_sans_vide(BAT_CODE_MAX, 3, "chutes brèves (retour)");
    END LOOP;
    tenir_sans_vide(BAT_CODE_MIN - 1, FILTRE_TB - 1, "filtre c");
    attendre_cycles(1);
    verifier(batt_vide = '1', "batt_vide absent après exactement FILTRE_BATT_CYCLES_G cycles sous le seuil");

    ----------------------------------------------------------------------------
    -- 6. Batterie vide mémorisée et clignotement (voir verifier_clignotement).
    ----------------------------------------------------------------------------
    verifier_clignotement("première déclaration");
    -- Bon code pendant longtemps : toujours vide, toujours 50 %.
    compter_hauts(led7, 40 * CLIGNO_TB, h);
    verifier(batt_vide = '1', "batt_vide retombé sans reset");
    verifier(h = 20 * CLIGNO_TB, "clignotement : " & INTEGER'IMAGE(h) & " cycles allumés sur " &
             INTEGER'IMAGE(40 * CLIGNO_TB) & " au lieu de 50 %");

    ----------------------------------------------------------------------------
    -- 7. Reset asynchrone pendant le clignotement : retour à la PWM (9 V ->
    --    100 %), batt_vide à '0'. Le reset tombe au milieu d'une phase
    --    éteinte, compteur à mi-course : un compteur ou une phase non remis
    --    à zéro fausserait le clignotement suivant (contrôlé à l'étape 8).
    ----------------------------------------------------------------------------
    WAIT UNTIL FALLING_EDGE(led7);
    attendre_cycles(CLIGNO_TB / 2);
    verifier(led7 = '0', "reset de l'étape 7 hors de la phase éteinte");
    faire_reset(10 us);
    tester_code(BAT_CODE_MAX, "après reset");

    -- Le reset remet aussi le générateur PWM à zéro : avec une consigne de
    -- 10 %, reset pendant l'impulsion ; après le relâchement, la consigne
    -- figée vaut 0 jusqu'à la fin de la première période, donc LED7 reste
    -- éteinte pendant P cycles (sans reset, l'ancienne consigne de 10 %
    -- ressortirait aussitôt).
    tester_code(BAT_CODE_MIN, "avant reset de la PWM");
    WAIT UNTIL RISING_EDGE(led7);
    attendre_cycles(2);
    verifier(led7 = '1', "reset de la PWM hors de l'impulsion");
    faire_reset(10 us);
    compter_hauts(led7, P, h);
    verifier(h = 0, "PWM non remise à zéro par le reset : " & INTEGER'IMAGE(h) &
             " cycles allumés sur la première période");

    ----------------------------------------------------------------------------
    -- 8. Le reset remet aussi le décompte du filtre à zéro : FILTRE - 10
    --    cycles sous le seuil, reset, puis code nul (pas encore de mesure
    --    ADC) : batt_vide exactement au FILTRE-ième front après le reset.
    ----------------------------------------------------------------------------
    tenir_sans_vide(0, FILTRE_TB - 10, "avant reset");
    faire_reset(10 us);
    tenir_sans_vide(0, FILTRE_TB - 1, "code nul après reset");
    attendre_cycles(1);
    verifier(batt_vide = '1', "après reset, batt_vide absent au FILTRE-ième cycle sous le seuil");
    -- Clignotement après un reset survenu en plein clignotement (étape 7) :
    -- il repart sur une demi-période allumée complète.
    verifier_clignotement("après reset");

    ----------------------------------------------------------------------------
    -- 9. Exemplaire réel (génériques par défaut) : rapport cyclique à mi-plage,
    --    puis code nul après reset : pas de batterie vide avant 1 s
    --    (FILTRE_BATT_CYCLES fronts), puis clignotement 0,5 s / 0,5 s.
    ----------------------------------------------------------------------------
    code      <= TO_UNSIGNED(BAT_CODE_MAX, 12);
    code_reel <= TO_UNSIGNED(3061, 12);
    attendre_cycles(2 * P + 2);
    compter_hauts(led7_reel, P, d);
    verifier(d = duty_formule(3061), "u_reel : code 3061 -> " & INTEGER'IMAGE(d) & " %");

    code_reel <= (OTHERS => '0');
    faire_reset(10 us);
    t_rel := NOW;
    WAIT UNTIL batt_vide_reel = '1' FOR 1.2 sec;
    -- Le k-ième front montant suit le relâchement de 1 µs + (k - 1) x 2 µs.
    fronts := (NOW - t_rel - PERIODE_CLK / 2) / PERIODE_CLK + 1;
    verifier(batt_vide_reel = '1', "u_reel : batterie jamais déclarée vide");
    verifier(fronts = FILTRE_BATT_CYCLES, "u_reel : batt_vide après " & INTEGER'IMAGE(fronts) &
             " cycles au lieu de " & INTEGER'IMAGE(FILTRE_BATT_CYCLES) & " (1 s)");
    verifier(NOW - t_rel >= 999 ms, "u_reel : batterie déclarée vide avant 1 s");
    WAIT UNTIL RISING_EDGE(led7_reel) FOR 1.2 sec;
    t1 := NOW;
    WAIT UNTIL FALLING_EDGE(led7_reel) FOR 1.2 sec;
    t2 := NOW;
    WAIT UNTIL RISING_EDGE(led7_reel) FOR 1.2 sec;
    t3 := NOW;
    verifier(t2 - t1 = 500 ms, "u_reel : LED7 allumée " & TIME'IMAGE(t2 - t1) & " au lieu de 0,5 s");
    verifier(t3 - t2 = 500 ms, "u_reel : LED7 éteinte " & TIME'IMAGE(t3 - t2) & " au lieu de 0,5 s");

    -- Bilan.
    IF nb_echecs = 0 AND echecs_structure = 0 THEN
      REPORT "TEST OK : tb_batterie_monitor (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_batterie_monitor (" & INTEGER'IMAGE(nb_echecs + echecs_structure) &
             " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    std.env.finish;
  END PROCESS verification;

END ARCHITECTURE tb_batterie_monitor_arch;
