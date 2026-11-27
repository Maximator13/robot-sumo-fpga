--------------------------------------------------------------------------------
-- Fichier      : tb_circuit_reset.vhd
-- Entité       : tb_circuit_reset
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant du circuit de reset
--                (cahier des charges §6.1 : rst_b à '0' pendant 1 ms à la mise
--                sous tension et lors d'un appui sur BP1).
-- Principe     : Deux exemplaires du circuit tournent sur la même horloge
--                50 MHz :
--                - u_reel garde la durée réelle (50 000 cycles) : on vérifie
--                  que le reset de mise sous tension dure bien 1 ms ;
--                - u_court a une durée réduite (N = 40 cycles) pour tester
--                  vite les appuis, le maintien, les rebonds du bouton.
--                Processus de surveillance permanents :
--                - rst_b ne change qu'à l'instant d'un front montant de clk_in
--                  et chaque reset dure au moins N cycles ;
--                - contrôle de structure : rst_b est en permanence égal à la
--                  bascule interne rst_b_reg (lue par un nom externe
--                  VHDL-2008). Une sortie décodée de façon combinatoire
--                  (source possible de parasites sur le reset) diverge d'au
--                  moins un cycle, et la suppression de la bascule empêche
--                  l'élaboration : les deux cas font échouer le test. Une
--                  simulation sans délais ne peut pas montrer un parasite
--                  physique : c'est la structure "sortie = bascule" qui le
--                  garantit, et c'est elle qui est contrôlée.
--                - chien de garde : si le test n'est pas fini à 5 ms
--                  (module bloqué), "TEST KO" au lieu d'une simulation sans fin.
--                "TEST OK" n'est affiché que si aucun contrôle n'a échoué.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY tb_circuit_reset IS
END ENTITY tb_circuit_reset;

ARCHITECTURE tb_circuit_reset_arch OF tb_circuit_reset IS

  CONSTANT PERIODE_50M : TIME    := 20 ns;  -- période de l'oscillateur 50 MHz
  CONSTANT N_COURT     : NATURAL := 40;     -- durée de reset réduite pour u_court

  SIGNAL clk_in      : STD_LOGIC := '0';
  SIGNAL bp_reel     : STD_LOGIC := '1';    -- BP1 de u_reel, jamais appuyé
  SIGNAL bp_court    : STD_LOGIC := '1';    -- BP1 de u_court
  SIGNAL rst_b_reel  : STD_LOGIC;
  SIGNAL rst_b_court : STD_LOGIC;
  SIGNAL fin_simu    : BOOLEAN := FALSE;
  SIGNAL num_front   : NATURAL := 0;        -- numéro du dernier front montant de clk_in

  -- Compteurs d'échecs des processus de surveillance (lus par le bilan).
  -- Un signal par processus (type non résolu : un seul pilote chacun).
  SIGNAL echecs_surveillance : NATURAL := 0;
  SIGNAL echecs_structure    : NATURAL := 0;

BEGIN

  -- Exemplaire à durée réelle (1 ms).
  u_reel : ENTITY WORK.circuit_reset
    PORT MAP (clk_in => clk_in, bp_in => bp_reel, rst_b => rst_b_reel);

  -- Exemplaire à durée réduite (N_COURT cycles).
  u_court : ENTITY WORK.circuit_reset
    GENERIC MAP (RESET_CYCLES_G => N_COURT)
    PORT MAP (clk_in => clk_in, bp_in => bp_court, rst_b => rst_b_court);

  -- Horloge 50 MHz : premier front montant à 10 ns.
  horloge : PROCESS
  BEGIN
    WHILE NOT fin_simu LOOP
      clk_in <= '0'; WAIT FOR PERIODE_50M / 2;
      clk_in <= '1'; WAIT FOR PERIODE_50M / 2;
    END LOOP;
    WAIT;
  END PROCESS horloge;

  comptage_fronts : PROCESS (clk_in)
  BEGIN
    IF RISING_EDGE(clk_in) THEN
      num_front <= num_front + 1;
    END IF;
  END PROCESS comptage_fronts;

  -- Contrôle de structure : la sortie rst_b de chaque exemplaire doit être
  -- à tout instant la bascule interne rst_b_reg (vérifié à chaque front de
  -- clk_in, 1 ns après, une fois les deltas stabilisés).
  sortie_registree : PROCESS
    ALIAS reg_court IS << SIGNAL .tb_circuit_reset.u_court.rst_b_reg : STD_LOGIC >>;
    ALIAS reg_reel  IS << SIGNAL .tb_circuit_reset.u_reel.rst_b_reg  : STD_LOGIC >>;
  BEGIN
    WAIT ON clk_in;
    WAIT FOR 1 ns;
    IF rst_b_court /= reg_court OR rst_b_reel /= reg_reel THEN
      echecs_structure <= echecs_structure + 1;
      REPORT "ECHEC : rst_b n'est pas la sortie directe de la bascule rst_b_reg" SEVERITY ERROR;
    END IF;
  END PROCESS sortie_registree;

  -- Chien de garde : le test complet dure environ 1 ms ; à 5 ms il est bloqué.
  chien_de_garde : PROCESS
  BEGIN
    WAIT FOR 5 ms;
    IF NOT fin_simu THEN
      REPORT "TEST KO : tb_circuit_reset bloqué (délai de 5 ms dépassé)" SEVERITY FAILURE;
    END IF;
    WAIT;
  END PROCESS chien_de_garde;

  -- Surveillance permanente de u_court : chaque changement de rst_b doit
  -- avoir lieu à l'instant d'un front montant de clk_in, et chaque période
  -- de reset doit durer au moins N_COURT cycles.
  surveillance : PROCESS
    VARIABLE front_chute : NATURAL := 0;
  BEGIN
    WAIT ON rst_b_court;
    -- À t = 0 la sortie passe de 'U' à sa valeur de mise sous tension : ce
    -- n'est pas un changement de fonctionnement, on ne le contrôle pas.
    IF NOW > 0 ns AND NOT (clk_in = '1' AND clk_in'LAST_EVENT = 0 ns) THEN
      echecs_surveillance <= echecs_surveillance + 1;
      REPORT "ECHEC : rst_b a changé hors d'un front d'horloge (parasite)" SEVERITY ERROR;
    END IF;
    IF rst_b_court = '0' THEN
      front_chute := num_front;
    ELSIF rst_b_court = '1' AND num_front - front_chute < N_COURT THEN
      echecs_surveillance <= echecs_surveillance + 1;
      REPORT "ECHEC : reset de seulement " & INTEGER'IMAGE(num_front - front_chute) & " cycles" SEVERITY ERROR;
    END IF;
  END PROCESS surveillance;

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

    -- Attend que rst_b_court passe à la valeur donnée, au plus max_fronts
    -- fronts d'horloge ; renvoie le nombre de fronts écoulés.
    PROCEDURE attendre_rst_court (valeur : STD_LOGIC; max_fronts : NATURAL; nb_fronts : OUT NATURAL) IS
      VARIABLE debut : NATURAL;
    BEGIN
      debut := num_front;
      WHILE rst_b_court /= valeur AND num_front - debut <= max_fronts LOOP
        WAIT UNTIL RISING_EDGE(clk_in);
        WAIT FOR 1 ns;
      END LOOP;
      nb_fronts := num_front - debut;
    END PROCEDURE attendre_rst_court;

    VARIABLE nb_fronts : NATURAL;

  BEGIN
    ----------------------------------------------------------------------------
    -- 1. Mise sous tension : reset actif immédiatement (valeur initiale).
    ----------------------------------------------------------------------------
    WAIT FOR 1 ns;
    verifier(rst_b_reel = '0' AND rst_b_court = '0', "reset inactif à la mise sous tension");

    -- u_court : libéré au front N+1 (N fronts de comptage + 1 front de sortie).
    attendre_rst_court('1', N_COURT + 10, nb_fronts);
    verifier(nb_fronts = N_COURT + 1,
             "reset de mise sous tension de " & INTEGER'IMAGE(nb_fronts) & " fronts au lieu de " & INTEGER'IMAGE(N_COURT + 1));

    ----------------------------------------------------------------------------
    -- 2. Appui sur BP1 : reset actif 3 fronts plus tard (2 bascules de
    --    synchronisation + bascule de sortie), maintenu tant que BP1 est appuyé.
    ----------------------------------------------------------------------------
    WAIT FOR 20 * PERIODE_50M;
    verifier(rst_b_court = '1', "reset relancé sans appui");
    WAIT FOR 7 ns;                       -- appui hors front d'horloge
    bp_court <= '0';
    attendre_rst_court('0', 10, nb_fronts);
    verifier(nb_fronts = 3, "reset activé " & INTEGER'IMAGE(nb_fronts) & " fronts après l'appui au lieu de 3");

    -- Bouton maintenu 5 fois la durée du reset : rst_b doit rester à '0'.
    FOR i IN 1 TO 5 * N_COURT LOOP
      WAIT UNTIL RISING_EDGE(clk_in);
      WAIT FOR 1 ns;
      verifier(rst_b_court = '0', "reset relâché alors que BP1 est encore appuyé");
    END LOOP;

    ----------------------------------------------------------------------------
    -- 3. Relâchement : rst_b repasse à '1' exactement N+3 fronts plus tard
    --    (2 de synchronisation + N de comptage + 1 de sortie), donc 1 ms après
    --    le relâchement avec la durée réelle.
    ----------------------------------------------------------------------------
    WAIT FOR 4 ns;
    bp_court <= '1';
    attendre_rst_court('1', N_COURT + 10, nb_fronts);
    verifier(nb_fronts = N_COURT + 3,
             "relâchement : reset libéré après " & INTEGER'IMAGE(nb_fronts) & " fronts au lieu de " & INTEGER'IMAGE(N_COURT + 3));

    ----------------------------------------------------------------------------
    -- 4. Rebonds : impulsions de BP1 pendant 10 cycles, puis bouton relâché.
    --    Le décompte doit repartir du dernier rebond capturé.
    ----------------------------------------------------------------------------
    WAIT FOR 20 * PERIODE_50M + 3 ns;
    FOR rebond IN 1 TO 5 LOOP
      bp_court <= '0'; WAIT FOR 30 ns;   -- appui qui couvre au moins un front
      bp_court <= '1'; WAIT FOR 13 ns;
    END LOOP;
    -- Dernier appui capturé : le bouton est relâché depuis le front suivant.
    verifier(rst_b_court = '0', "les rebonds n'ont pas déclenché le reset");
    attendre_rst_court('1', N_COURT + 10, nb_fronts);
    verifier(nb_fronts >= N_COURT AND nb_fronts <= N_COURT + 3,
             "après rebonds : reset libéré après " & INTEGER'IMAGE(nb_fronts) & " fronts");

    ----------------------------------------------------------------------------
    -- 5. Impulsion parasite très courte entre deux fronts : non capturée par
    --    la synchronisation, elle ne doit pas relancer de reset.
    ----------------------------------------------------------------------------
    WAIT UNTIL RISING_EDGE(clk_in);
    WAIT FOR 3 ns;
    bp_court <= '0'; WAIT FOR 4 ns; bp_court <= '1';
    FOR i IN 1 TO 10 LOOP
      WAIT UNTIL RISING_EDGE(clk_in);
      WAIT FOR 1 ns;
      verifier(rst_b_court = '1', "une impulsion de 4 ns entre deux fronts a déclenché le reset");
    END LOOP;

    ----------------------------------------------------------------------------
    -- 6. Durée réelle : u_reel libère le reset au front 50 001, soit à
    --    10 ns + 50 000 x 20 ns = 1,00001 ms : au moins 1 ms comme exigé.
    ----------------------------------------------------------------------------
    -- Attente bornée : si u_reel ne relâche jamais le reset, échec explicite.
    IF rst_b_reel /= '1' THEN
      WAIT UNTIL rst_b_reel = '1' FOR 1 ms + 100 ns;
    END IF;
    verifier(rst_b_reel = '1', "le reset réel n'est jamais relâché");
    verifier(num_front = RESET_CYCLES + 1,
             "reset réel libéré au front " & INTEGER'IMAGE(num_front) & " au lieu de " & INTEGER'IMAGE(RESET_CYCLES + 1));
    verifier(NOW >= 1 ms AND NOW < 1 ms + 100 ns,
             "reset réel de " & TIME'IMAGE(NOW) & " au lieu de 1 ms");
    -- Pendant tout ce temps u_reel n'a jamais relâché le reset avant 1 ms
    -- (WAIT UNTIL ci-dessus s'est déclenché au premier passage à '1').

    -- Bilan (contrôles directs + processus de surveillance).
    WAIT FOR 1 ns;
    IF nb_echecs = 0 AND echecs_surveillance = 0 AND echecs_structure = 0 THEN
      REPORT "TEST OK : tb_circuit_reset (" & INTEGER'IMAGE(nb_controles) & " controles + surveillance permanente)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_circuit_reset (" & INTEGER'IMAGE(nb_echecs + echecs_surveillance + echecs_structure) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    std.env.finish;
  END PROCESS verification;

END ARCHITECTURE tb_circuit_reset_arch;
