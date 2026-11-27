--------------------------------------------------------------------------------
-- Fichier      : tb_diviseur_frequence.vhd
-- Entité       : tb_diviseur_frequence
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Banc de test auto-vérifiant du diviseur de fréquence
--                50 MHz -> 500 kHz (cahier des charges §6.2).
-- Principe     : Une horloge de 50 MHz (période 20 ns) alimente le diviseur.
--                Un processus compte les fronts montants de clk_in ; à chaque
--                front de clk_500KHz il mémorise le numéro du front d'entrée,
--                ce qui permet de mesurer exactement, en périodes de 50 MHz :
--                la période de sortie (100), le temps à '1' (50) et à '0'
--                (50), le délai du premier front après le reset (50). La
--                période est aussi mesurée en temps simulé (2 µs).
--                Contrôles aussi : sortie à '0' pendant le reset, retour
--                immédiat à '0' sur un reset asynchrone en cours de marche.
--                Contrôle de structure permanent : la sortie est à tout
--                instant la bascule interne clk_500KHz_reg (nom externe
--                VHDL-2008), car une horloge décodée de façon combinatoire
--                pourrait porter des parasites.
--                Chien de garde : si le test n'est pas fini à 200 µs (sortie
--                figée par exemple), "TEST KO" au lieu d'une simulation sans fin.
--                "TEST OK" n'est affiché que si aucun contrôle n'a échoué.
-- Entrées      : aucune (banc de test)
-- Sorties      : aucune (banc de test)
-- Génériques   : aucun
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY tb_diviseur_frequence IS
END ENTITY tb_diviseur_frequence;

ARCHITECTURE tb_diviseur_frequence_arch OF tb_diviseur_frequence IS

  CONSTANT PERIODE_50M : TIME := 20 ns;   -- période de l'oscillateur 50 MHz

  SIGNAL clk_in     : STD_LOGIC := '0';
  SIGNAL rst_in_b   : STD_LOGIC := '0';
  SIGNAL clk_500KHz : STD_LOGIC;
  SIGNAL fin_simu   : BOOLEAN := FALSE;

  -- Numéro du dernier front montant de clk_in (compté depuis le début).
  SIGNAL num_front  : NATURAL := 0;

BEGIN

  -- Composant testé.
  u_diviseur_frequence : ENTITY WORK.diviseur_frequence
    PORT MAP (
      clk_in     => clk_in,
      rst_in_b   => rst_in_b,
      clk_500KHz => clk_500KHz
    );

  -- Génération de l'horloge 50 MHz, arrêtée en fin de simulation.
  horloge : PROCESS
  BEGIN
    WHILE NOT fin_simu LOOP
      clk_in <= '0'; WAIT FOR PERIODE_50M / 2;
      clk_in <= '1'; WAIT FOR PERIODE_50M / 2;
    END LOOP;
    WAIT;
  END PROCESS horloge;

  -- Contrôle de structure : la sortie doit être la bascule clk_500KHz_reg.
  sortie_registree : PROCESS
    ALIAS reg_sortie IS << SIGNAL .tb_diviseur_frequence.u_diviseur_frequence.clk_500KHz_reg : STD_LOGIC >>;
  BEGIN
    WAIT ON clk_in;
    WAIT FOR 1 ns;
    ASSERT clk_500KHz = reg_sortie
      REPORT "ECHEC : clk_500KHz n'est pas la sortie directe de la bascule clk_500KHz_reg" SEVERITY FAILURE;
  END PROCESS sortie_registree;

  -- Chien de garde : le test complet dure environ 30 µs.
  chien_de_garde : PROCESS
  BEGIN
    WAIT FOR 200 us;
    IF NOT fin_simu THEN
      REPORT "TEST KO : tb_diviseur_frequence bloqué (délai de 200 us dépassé)" SEVERITY FAILURE;
    END IF;
    WAIT;
  END PROCESS chien_de_garde;

  -- Numérotation des fronts montants de clk_in (référence de temps du test).
  comptage_fronts : PROCESS (clk_in)
  BEGIN
    IF RISING_EDGE(clk_in) THEN
      num_front <= num_front + 1;
    END IF;
  END PROCESS comptage_fronts;

  verification : PROCESS
    VARIABLE nb_controles : NATURAL := 0;
    VARIABLE nb_echecs    : NATURAL := 0;
    VARIABLE front_haut_precedent : NATURAL;  -- num_front au front montant précédent de la sortie
    VARIABLE front_haut   : NATURAL;          -- num_front au front montant courant
    VARIABLE front_bas    : NATURAL;          -- num_front au front descendant
    VARIABLE front_reset  : NATURAL;          -- num_front au relâchement du reset
    VARIABLE instant_front : TIME;            -- date d'un front montant de la sortie

    PROCEDURE verifier (condition : BOOLEAN; message : STRING) IS
    BEGIN
      nb_controles := nb_controles + 1;
      IF NOT condition THEN
        nb_echecs := nb_echecs + 1;
      END IF;
      ASSERT condition REPORT "ECHEC : " & message SEVERITY ERROR;
    END PROCEDURE verifier;

  BEGIN
    -- 1. Reset actif : la sortie doit rester à '0' quels que soient les fronts.
    FOR i IN 1 TO 120 LOOP
      WAIT UNTIL RISING_EDGE(clk_in);
      WAIT FOR 1 ns;
      verifier(clk_500KHz = '0', "sortie non nulle pendant le reset");
    END LOOP;

    -- 2. Relâchement du reset hors front d'horloge, puis premier front montant
    --    de la sortie : il doit arriver 50 fronts de clk_in plus tard.
    WAIT FOR 3 ns;
    rst_in_b    <= '1';
    front_reset := num_front;
    WAIT UNTIL RISING_EDGE(clk_500KHz);
    verifier(num_front - front_reset = 50,
             "premier front après reset au front " & INTEGER'IMAGE(num_front - front_reset) & " au lieu de 50");
    front_haut_precedent := num_front;

    -- 3. Sur 10 périodes : période de 100 cycles, 50 à '1', 50 à '0'.
    FOR periode IN 1 TO 10 LOOP
      WAIT UNTIL FALLING_EDGE(clk_500KHz);
      front_bas := num_front;
      WAIT UNTIL RISING_EDGE(clk_500KHz);
      front_haut := num_front;
      verifier(front_bas - front_haut_precedent = 50,
               "temps haut de " & INTEGER'IMAGE(front_bas - front_haut_precedent) & " cycles au lieu de 50");
      verifier(front_haut - front_bas = 50,
               "temps bas de " & INTEGER'IMAGE(front_haut - front_bas) & " cycles au lieu de 50");
      verifier(front_haut - front_haut_precedent = CLK_IN_HZ / CLK_SYS_HZ,
               "période de " & INTEGER'IMAGE(front_haut - front_haut_precedent) & " cycles au lieu de 100");
      front_haut_precedent := front_haut;
    END LOOP;

    -- 4. Mesure en temps simulé : une période de sortie doit durer 2 µs
    --    (500 kHz), indépendamment du comptage de fronts ci-dessus.
    WAIT UNTIL RISING_EDGE(clk_500KHz);
    instant_front := NOW;
    WAIT UNTIL RISING_EDGE(clk_500KHz);
    verifier(NOW - instant_front = 2 us,
             "période mesurée de " & TIME'IMAGE(NOW - instant_front) & " au lieu de 2 us");

    -- 5. Reset asynchrone pendant que la sortie est à '1' : retour à '0'
    --    immédiat, sans attendre un front de clk_in.
    WAIT FOR 10 * PERIODE_50M + 5 ns;
    verifier(clk_500KHz = '1', "la sortie devrait être à '1' 10 cycles après un front montant");
    rst_in_b <= '0';
    WAIT FOR 1 ns;
    verifier(clk_500KHz = '0', "le reset asynchrone n'a pas forcé la sortie à '0'");

    -- 6. Après ce second reset, le diviseur repart proprement (50 cycles).
    WAIT FOR 4 * PERIODE_50M;
    rst_in_b    <= '1';
    front_reset := num_front;
    WAIT UNTIL RISING_EDGE(clk_500KHz);
    verifier(num_front - front_reset = 50, "redémarrage après reset incorrect");

    -- Bilan.
    IF nb_echecs = 0 THEN
      REPORT "TEST OK : tb_diviseur_frequence (" & INTEGER'IMAGE(nb_controles) & " controles)" SEVERITY NOTE;
    ELSE
      REPORT "TEST KO : tb_diviseur_frequence (" & INTEGER'IMAGE(nb_echecs) & " echec(s))" SEVERITY FAILURE;
    END IF;
    fin_simu <= TRUE;
    std.env.finish;
  END PROCESS verification;

END ARCHITECTURE tb_diviseur_frequence_arch;
