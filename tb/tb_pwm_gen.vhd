--------------------------------------------------------------------------------
-- Fichier      : tb_pwm_gen.vhd
-- Entité       : tb_pwm_gen
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant du générateur PWM
--                (PWM moteurs 5 kHz imposée par le cahier des charges §6).
-- Principe     : Horloge système réelle de 500 kHz (2 µs). Deux exemplaires :
--                - u_pwm : période réelle de 100 cycles (5 kHz) ;
--                - u_pwm_10 : période de 10 cycles, pour vérifier que le
--                  générique est bien respecté.
--                Pour chaque consigne, on attend que la consigne soit prise
--                en compte (2 périodes), puis on échantillonne la sortie à
--                chaque front d'horloge sur plusieurs périodes : nombre de
--                cycles à '1', nombre d'impulsions, longueur de chaque
--                impulsion. On vérifie aussi la fréquence (200 µs), la
--                saturation des consignes > 100, l'absence d'impulsion
--                tronquée quand la consigne change en cours de période, et le
--                reset asynchrone. Chien de garde à 50 ms (module bloqué).
--                "TEST OK" seulement si 0 échec.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY tb_pwm_gen IS
END ENTITY tb_pwm_gen;

ARCHITECTURE tb_pwm_gen_arch OF tb_pwm_gen IS

  CONSTANT PERIODE_CLK : TIME    := 2 us;   -- horloge système 500 kHz
  CONSTANT P           : NATURAL := PWM_PERIODE_CYCLES;  -- 100 cycles

  SIGNAL clk      : STD_LOGIC := '0';
  SIGNAL rst_b    : STD_LOGIC := '0';
  SIGNAL duty     : t_duty    := (OTHERS => '0');
  SIGNAL pwm      : STD_LOGIC;
  SIGNAL duty_10  : t_duty    := (OTHERS => '0');
  SIGNAL pwm_10   : STD_LOGIC;
  SIGNAL fin_simu : BOOLEAN   := FALSE;

BEGIN

  -- Exemplaire à la période réelle (5 kHz).
  u_pwm : ENTITY WORK.pwm_gen
    PORT MAP (clk => clk, rst_b => rst_b, duty => duty, pwm => pwm);

  -- Exemplaire à période de 10 cycles (contrôle du générique).
  u_pwm_10 : ENTITY WORK.pwm_gen
    GENERIC MAP (PERIODE_CYCLES => 10)
    PORT MAP (clk => clk, rst_b => rst_b, duty => duty_10, pwm => pwm_10);

  horloge : PROCESS
  BEGIN
    WHILE NOT fin_simu LOOP
      clk <= '0'; WAIT FOR PERIODE_CLK / 2;
      clk <= '1'; WAIT FOR PERIODE_CLK / 2;
    END LOOP;
    WAIT;
  END PROCESS horloge;

  -- Chien de garde : le test complet dure environ 13 ms.
  chien_de_garde : PROCESS
  BEGIN
    WAIT FOR 50 ms;
    IF NOT fin_simu THEN
      REPORT "TEST KO : tb_pwm_gen bloqué (délai de 50 ms dépassé)" SEVERITY FAILURE;
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

    -- Échantillonne un signal PWM sur nb_cycles cycles consécutifs et renvoie
    -- le nombre de cycles à '1', le nombre de fronts montants, et les
    -- longueurs min/max des impulsions complètes observées.
    PROCEDURE mesurer (SIGNAL s : IN STD_LOGIC; nb_cycles : NATURAL;
                       nb_hauts, nb_montees, imp_min, imp_max : OUT NATURAL) IS
      VARIABLE precedent : STD_LOGIC;
      VARIABLE longueur  : NATURAL := 0;
      VARIABLE hauts, montees : NATURAL := 0;
      VARIABLE lmin : NATURAL := NATURAL'HIGH;
      VARIABLE lmax : NATURAL := 0;
      VARIABLE dans_impulsion_complete : BOOLEAN := FALSE;
    BEGIN
      precedent := s;
      FOR i IN 1 TO nb_cycles LOOP
        WAIT UNTIL RISING_EDGE(clk);
        WAIT FOR 1 ns;
        IF s = '1' THEN
          hauts := hauts + 1;
        END IF;
        IF precedent = '0' AND s = '1' THEN
          -- Début d'impulsion observé en entier.
          montees  := montees + 1;
          longueur := 1;
          dans_impulsion_complete := TRUE;
        ELSIF precedent = '1' AND s = '1' THEN
          longueur := longueur + 1;
        ELSIF precedent = '1' AND s = '0' AND dans_impulsion_complete THEN
          -- Fin d'une impulsion dont on a vu le début : on enregistre sa longueur.
          IF longueur < lmin THEN lmin := longueur; END IF;
          IF longueur > lmax THEN lmax := longueur; END IF;
          dans_impulsion_complete := FALSE;
        END IF;
        precedent := s;
      END LOOP;
      nb_hauts := hauts; nb_montees := montees; imp_min := lmin; imp_max := lmax;
    END PROCEDURE mesurer;

    -- Applique une consigne au générateur 5 kHz, attend sa prise en compte,
    -- puis vérifie 3 périodes complètes.
    PROCEDURE tester_consigne (consigne : NATURAL) IS
      VARIABLE attendu : NATURAL;
      VARIABLE hauts, montees, lmin, lmax : NATURAL;
    BEGIN
      duty <= TO_UNSIGNED(consigne, t_duty'LENGTH);
      attendre_cycles(2 * P + 1);     -- prise en compte à la fin de la période en cours
      IF consigne > P THEN attendu := P; ELSE attendu := consigne; END IF;
      mesurer(pwm, 3 * P, hauts, montees, lmin, lmax);
      verifier(hauts = 3 * attendu,
               "consigne " & INTEGER'IMAGE(consigne) & " : " & INTEGER'IMAGE(hauts) &
               " cycles à 1 sur 3 périodes au lieu de " & INTEGER'IMAGE(3 * attendu));
      IF attendu > 0 AND attendu < P THEN
        -- Une seule impulsion par période, toujours de la même longueur.
        verifier(montees = 3, "consigne " & INTEGER'IMAGE(consigne) & " : " &
                 INTEGER'IMAGE(montees) & " impulsions sur 3 périodes au lieu de 3");
        verifier(lmin = attendu AND lmax = attendu, "consigne " & INTEGER'IMAGE(consigne) &
                 " : impulsions de " & INTEGER'IMAGE(lmin) & " à " & INTEGER'IMAGE(lmax) & " cycles");
      ELSE
        -- 0 % ou 100 % : sortie constante, aucune transition.
        verifier(montees = 0, "consigne " & INTEGER'IMAGE(consigne) & " : la sortie devrait être constante");
      END IF;
    END PROCEDURE tester_consigne;

    VARIABLE hauts, montees, lmin, lmax : NATURAL;
    VARIABLE instant : TIME;
    VARIABLE longueur : NATURAL;

  BEGIN
    ----------------------------------------------------------------------------
    -- 1. Pendant le reset, sortie à '0' même avec une consigne de 100 %.
    ----------------------------------------------------------------------------
    duty <= TO_UNSIGNED(100, t_duty'LENGTH);
    FOR i IN 1 TO 2 * P LOOP
      WAIT UNTIL RISING_EDGE(clk);
      WAIT FOR 1 ns;
      verifier(pwm = '0', "sortie non nulle pendant le reset");
    END LOOP;
    duty <= (OTHERS => '0');
    WAIT FOR 300 ns;
    rst_b <= '1';

    ----------------------------------------------------------------------------
    -- 2. Rapport cyclique : cas limites, valeurs courantes, saturation.
    ----------------------------------------------------------------------------
    tester_consigne(0);
    tester_consigne(1);
    tester_consigne(25);
    tester_consigne(50);
    tester_consigne(99);
    tester_consigne(100);
    tester_consigne(101);   -- au-delà de 100 : saturée à 100 %
    tester_consigne(127);   -- valeur maximale de t_duty
    tester_consigne(VITESSE_RECHERCHE);

    ----------------------------------------------------------------------------
    -- 3. Fréquence : 200 µs entre deux fronts montants (5 kHz).
    ----------------------------------------------------------------------------
    duty <= TO_UNSIGNED(50, t_duty'LENGTH);
    attendre_cycles(2 * P + 1);
    WAIT UNTIL RISING_EDGE(pwm);
    instant := NOW;
    WAIT UNTIL RISING_EDGE(pwm);
    verifier(NOW - instant = 200 us,
             "période PWM de " & TIME'IMAGE(NOW - instant) & " au lieu de 200 us (5 kHz)");

    ----------------------------------------------------------------------------
    -- 4. Consigne abaissée en cours d'impulsion (50 -> 10) : l'impulsion en
    --    cours n'est pas tronquée, la suivante fait 10 cycles.
    ----------------------------------------------------------------------------
    WAIT UNTIL RISING_EDGE(pwm);
    attendre_cycles(5);
    duty <= TO_UNSIGNED(10, t_duty'LENGTH);
    -- Déjà vus à '1' : le cycle du front montant + les 5 cycles suivants.
    longueur := 6;
    WHILE pwm = '1' LOOP
      attendre_cycles(1);
      IF pwm = '1' THEN longueur := longueur + 1; END IF;
    END LOOP;
    verifier(longueur = 50, "impulsion tronquée par un changement de consigne : " &
             INTEGER'IMAGE(longueur) & " cycles au lieu de 50");
    mesurer(pwm, 2 * P, hauts, montees, lmin, lmax);
    verifier(lmin = 10 AND lmax = 10, "après changement, impulsions de " &
             INTEGER'IMAGE(lmin) & " à " & INTEGER'IMAGE(lmax) & " cycles au lieu de 10");

    ----------------------------------------------------------------------------
    -- 5. Consigne relevée juste après la fin de l'impulsion (10 -> 99) : pas
    --    de seconde impulsion dans la même période. Avec 99, une prise en
    --    compte prématurée à n'importe quel cycle entre 11 et 98 créerait une
    --    impulsion visible, donc le test couvre toute la fin de période.
    ----------------------------------------------------------------------------
    WAIT UNTIL FALLING_EDGE(pwm);
    WAIT FOR 1 ns;                    -- cycle 11 de la période
    duty <= TO_UNSIGNED(99, t_duty'LENGTH);
    hauts := 0;
    FOR i IN 1 TO P - 12 LOOP         -- reste de la période en cours (cycles 11 à 98)
      attendre_cycles(1);
      IF pwm = '1' THEN hauts := hauts + 1; END IF;
    END LOOP;
    verifier(hauts = 0, "seconde impulsion dans la même période après hausse de consigne");
    attendre_cycles(P);
    mesurer(pwm, 2 * P, hauts, montees, lmin, lmax);
    verifier(lmin = 99 AND lmax = 99, "après hausse, impulsions de " &
             INTEGER'IMAGE(lmin) & " à " & INTEGER'IMAGE(lmax) & " cycles au lieu de 99");

    ----------------------------------------------------------------------------
    -- 6. Générique PERIODE_CYCLES = 10 : 3 cycles sur 10, et saturation à 10.
    ----------------------------------------------------------------------------
    duty_10 <= TO_UNSIGNED(3, t_duty'LENGTH);
    attendre_cycles(21);
    mesurer(pwm_10, 50, hauts, montees, lmin, lmax);
    verifier(hauts = 15 AND montees = 5 AND lmin = 3 AND lmax = 3,
             "période 10 / consigne 3 : " & INTEGER'IMAGE(hauts) & " cycles à 1 sur 50 au lieu de 15");
    duty_10 <= TO_UNSIGNED(15, t_duty'LENGTH);
    attendre_cycles(21);
    mesurer(pwm_10, 50, hauts, montees, lmin, lmax);
    verifier(hauts = 50, "période 10 / consigne 15 : saturation à 100 % attendue");

    ----------------------------------------------------------------------------
    -- 7. Reset asynchrone en cours d'impulsion : sortie à '0' immédiatement,
    --    puis le générateur repart à 0 % (consigne figée remise à zéro).
    ----------------------------------------------------------------------------
    WAIT UNTIL RISING_EDGE(pwm);
    WAIT FOR 5 us;
    rst_b <= '0';
    WAIT FOR 1 ns;
    verifier(pwm = '0', "le reset asynchrone n'a pas coupé la sortie");
    WAIT FOR 10 us;
    rst_b <= '1';
    -- Toute la première période après reset doit rester à '0' : la consigne
    -- figée (99 % avant le reset) doit avoir été remise à zéro.
    hauts := 0;
    FOR i IN 1 TO P - 1 LOOP
      attendre_cycles(1);
      IF pwm = '1' THEN hauts := hauts + 1; END IF;
    END LOOP;
    verifier(hauts = 0, "sortie active " & INTEGER'IMAGE(hauts) &
             " cycles dans la première période après reset (consigne non remise à zéro)");
    -- Puis la consigne de 99 % est reprise normalement.
    attendre_cycles(P);
    mesurer(pwm, 2 * P, hauts, montees, lmin, lmax);
    verifier(lmin = 99 AND lmax = 99, "après reset, la consigne de 99 % n'est pas reprise");

    -- Bilan.
    IF nb_echecs = 0 THEN
      REPORT "TEST OK : tb_pwm_gen (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_pwm_gen (" & INTEGER'IMAGE(nb_echecs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    std.env.finish;
  END PROCESS verification;

END ARCHITECTURE tb_pwm_gen_arch;
