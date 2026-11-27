--------------------------------------------------------------------------------
-- Fichier      : adc128s022_ctrl.vhd
-- Entité       : adc128s022_ctrl
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Description  : Contrôleur SPI du convertisseur analogique-numérique
--                ADC128S022 de la DE0-Nano (8 voies, 12 bits). Il lit en
--                boucle les 8 voies et fournit en permanence la dernière
--                valeur de chacune : tension batterie (voie 0) et capteurs de
--                sol (voies 1 à 5). Même rôle que l'IP « ADC Controller for
--                DE-series Boards » du TP2, mais simulable.
-- Principe     : Cadencé à 50 MHz. SCLK = 50 MHz / DIV_SCLK = 3,125 MHz
--                (maximum de l'ADC : 3,2 MHz). Chaque trame :
--                1) CS passe à '0' (SCLK au repos à '1'), une demi-période
--                   avant le premier front descendant de SCLK ;
--                2) 16 périodes de SCLK : SCLK à '0' pendant la première
--                   moitié, à '1' pendant la seconde. Le contrôleur change
--                   DIN sur le front descendant et lit DOUT juste avant le
--                   front montant (DOUT a été mis à jour par l'ADC sur le
--                   front descendant) ;
--                3) DIN porte le mot de contrôle "00" & adresse & "000" :
--                   l'ADC lit l'adresse sur les fronts montants 3, 4 et 5 ;
--                4) DOUT donne 4 zéros puis D11..D0 : les 12 derniers bits
--                   lus forment la mesure ;
--                5) CS remonte pendant une demi-période, puis trame suivante.
--                Piège de l'ADC128S022 : l'adresse envoyée pendant une trame
--                choisit la voie convertie à la trame SUIVANTE. Le contrôleur
--                mémorise donc la voie demandée à la trame précédente
--                (canal_converti) pour ranger chaque mesure au bon endroit.
--                L'ADC n'a pas de reset : après un reset du FPGA il peut
--                convertir une voie quelconque. La première trame après un
--                reset sert seulement à envoyer l'adresse 0 ; son résultat
--                est ignoré.
--                Toutes les sorties sont des bascules. Une trame dure 17
--                périodes de SCLK (5,44 µs), un tour des 8 voies 43,5 µs.
-- Entrées      : clk      : horloge 50 MHz.
--                rst_b    : reset asynchrone actif à '0' (CS à '1', SCLK à '1').
--                adc_sdat : DOUT de l'ADC (PIN_A9).
-- Sorties      : adc_cs_n  : CS de l'ADC (PIN_A10).
--                adc_saddr : DIN de l'ADC (PIN_B10).
--                adc_sclk  : SCLK de l'ADC (PIN_B14), repos à '1'.
--                voies     : dernière valeur 12 bits de chaque voie (0 après
--                            reset, jusqu'à la première mesure).
--                mesure_ok : '1' pendant un cycle à chaque nouvelle mesure.
-- Génériques   : DIV_SCLK : période de SCLK en cycles de clk, PAIRE (défaut
--                16 : 3,125 MHz). La datasheet impose SCLK de 0,8 à 3,2 MHz
--                ET une cadence de 50 à 200 k échantillons/s ; avec 17
--                périodes de SCLK par trame, cadence = 50 MHz / (17 x
--                DIV_SCLK), d'où DIV_SCLK de 16 à 58.
--------------------------------------------------------------------------------
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE WORK.sumo_pkg.ALL;

ENTITY adc128s022_ctrl IS
  GENERIC (
    DIV_SCLK : NATURAL := 16   -- période SCLK en cycles de 50 MHz (paire)
  );
  PORT (
    clk       : IN  STD_LOGIC;     -- horloge 50 MHz
    rst_b     : IN  STD_LOGIC;     -- reset asynchrone, actif à '0'
    adc_cs_n  : OUT STD_LOGIC;     -- sélection de l'ADC, active à '0'
    adc_saddr : OUT STD_LOGIC;     -- donnée vers l'ADC (adresse de la voie)
    adc_sclk  : OUT STD_LOGIC;     -- horloge série
    adc_sdat  : IN  STD_LOGIC;     -- donnée venant de l'ADC (résultat)
    voies     : OUT t_voies_adc;   -- valeurs des 8 voies
    mesure_ok : OUT STD_LOGIC      -- impulsion : une voie vient d'être mise à jour
  );
END ENTITY adc128s022_ctrl;

ARCHITECTURE adc128s022_ctrl_arch OF adc128s022_ctrl IS

  -- Demi-période de SCLK en cycles de clk.
  CONSTANT DEMI_PERIODE : NATURAL := DIV_SCLK / 2;

  -- CS_HAUT : intervalle entre deux trames ; TRAME : 16 périodes de SCLK.
  TYPE t_etat_adc IS (CS_HAUT, TRAME);
  SIGNAL etat : t_etat_adc;

  -- Position dans la période de SCLK en cours.
  SIGNAL cpt_div      : NATURAL RANGE 0 TO DIV_SCLK - 1;
  -- Numéro de la période de SCLK dans la trame (0 à 15).
  SIGNAL num_periode  : NATURAL RANGE 0 TO 15;
  -- Les 16 bits lus sur DOUT pendant la trame (le premier lu à gauche).
  SIGNAL bits_lus     : STD_LOGIC_VECTOR(15 DOWNTO 0);
  -- Voie demandée pendant la trame en cours (convertie à la trame suivante).
  SIGNAL adresse_envoyee : UNSIGNED(2 DOWNTO 0);
  -- Voie réellement convertie pendant la trame en cours.
  SIGNAL canal_converti  : UNSIGNED(2 DOWNTO 0);
  -- '1' pendant la première trame après un reset (résultat ignoré).
  SIGNAL premiere_trame  : STD_LOGIC;

  -- Bascules de sortie.
  SIGNAL cs_n_reg      : STD_LOGIC;
  SIGNAL sclk_reg      : STD_LOGIC;
  SIGNAL saddr_reg     : STD_LOGIC;
  SIGNAL mesure_ok_reg : STD_LOGIC;
  SIGNAL voies_reg     : t_voies_adc;

  -- Bit du mot de contrôle à présenter sur DIN pendant la période p :
  -- mot = "00" & adresse(2..0) & "000", bit 7 pendant la période 0.
  FUNCTION bit_controle (adresse : UNSIGNED(2 DOWNTO 0); p : NATURAL) RETURN STD_LOGIC IS
    VARIABLE mot : STD_LOGIC_VECTOR(7 DOWNTO 0);
  BEGIN
    mot := "00" & STD_LOGIC_VECTOR(adresse) & "000";
    IF p <= 7 THEN
      RETURN mot(7 - p);
    ELSE
      RETURN '0';                 -- fronts 9 à 16 : DIN ignoré par l'ADC
    END IF;
  END FUNCTION bit_controle;

BEGIN

  sequencement_spi : PROCESS (clk, rst_b)
  BEGIN
    IF rst_b = '0' THEN
      -- Reset : ADC désélectionné, SCLK au repos, aucune mesure.
      etat            <= CS_HAUT;
      cpt_div         <= 0;
      num_periode     <= 0;
      bits_lus        <= (OTHERS => '0');
      adresse_envoyee <= (OTHERS => '0');
      canal_converti  <= (OTHERS => '0');
      premiere_trame  <= '1';
      cs_n_reg        <= '1';
      sclk_reg        <= '1';
      saddr_reg       <= '0';
      mesure_ok_reg   <= '0';
      voies_reg       <= (OTHERS => (OTHERS => '0'));
    ELSIF RISING_EDGE(clk) THEN
      -- Impulsion mesure_ok d'un seul cycle par défaut.
      mesure_ok_reg <= '0';

      -- Base de temps de SCLK.
      IF cpt_div = DIV_SCLK - 1 THEN
        cpt_div <= 0;
      ELSE
        cpt_div <= cpt_div + 1;
      END IF;

      CASE etat IS

        WHEN CS_HAUT =>
          -- Entre deux trames : SCLK au repos. CS descend au milieu de cette
          -- période, puis le premier front descendant de SCLK démarre la
          -- trame une demi-période plus tard (tCSS largement respecté).
          sclk_reg <= '1';
          IF cpt_div = DEMI_PERIODE - 1 THEN
            cs_n_reg <= '0';
          ELSIF cpt_div = DIV_SCLK - 1 THEN
            etat        <= TRAME;
            num_periode <= 0;
            sclk_reg    <= '0';                                  -- front descendant 1
            saddr_reg   <= bit_controle(adresse_envoyee, 0);     -- bit 7 du mot de contrôle
          END IF;

        WHEN TRAME =>
          IF cpt_div = DEMI_PERIODE - 1 THEN
            -- Fin de la demi-période basse : DOUT est stable depuis le front
            -- descendant, on le mémorise puis on remonte SCLK (front montant,
            -- où l'ADC lit DIN).
            bits_lus <= bits_lus(14 DOWNTO 0) & adc_sdat;
            sclk_reg <= '1';
          ELSIF cpt_div = DIV_SCLK - 1 THEN
            IF num_periode = 15 THEN
              -- Fin de trame : CS remonte, la mesure (12 derniers bits) est
              -- rangée sur la voie convertie, sauf après un reset.
              etat      <= CS_HAUT;
              cs_n_reg  <= '1';
              saddr_reg <= '0';
              IF premiere_trame = '0' THEN
                voies_reg(TO_INTEGER(canal_converti)) <= UNSIGNED(bits_lus(11 DOWNTO 0));
                mesure_ok_reg <= '1';
              END IF;
              premiere_trame  <= '0';
              -- La voie demandée maintenant sera convertie à la trame suivante.
              canal_converti  <= adresse_envoyee;
              adresse_envoyee <= adresse_envoyee + 1;            -- 7 + 1 = 0 : tour des 8 voies
            ELSE
              -- Période suivante : front descendant de SCLK et bit suivant
              -- du mot de contrôle sur DIN.
              num_periode <= num_periode + 1;
              sclk_reg    <= '0';
              saddr_reg   <= bit_controle(adresse_envoyee, num_periode + 1);
            END IF;
          END IF;

        WHEN OTHERS =>
          etat <= CS_HAUT;

      END CASE;
    END IF;
  END PROCESS sequencement_spi;

  -- Sorties directement issues de bascules.
  adc_cs_n  <= cs_n_reg;
  adc_sclk  <= sclk_reg;
  adc_saddr <= saddr_reg;
  voies     <= voies_reg;
  mesure_ok <= mesure_ok_reg;

END ARCHITECTURE adc128s022_ctrl_arch;
