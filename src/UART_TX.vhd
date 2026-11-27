--------------------------------------------------------------------------------
-- Fichier      : UART_TX.vhd
-- Entité       : UART_TX
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Origine      : Code fourni par l'enseignant (cahier des charges §5), issu de
--                https://www.nandland.com (module UART). La logique n'est PAS
--                modifiée : les commentaires d'origine (en anglais) ont été
--                remplacés par des commentaires en français, et la note
--                d'origine fausse sur o_TX_Done est corrigée ci-dessous.
-- Description  : Émetteur série asynchrone (UART) : envoie un octet au PC via
--                le module USB/série FT232R, au format 8N1 (1 bit de start,
--                8 bits de données poids faible en premier, pas de parité,
--                1 bit de stop). Dans le robot il transmet le message de
--                démarrage "Robot PRET !".
-- Principe     : Machine à états cadencée par i_Clk :
--                s_Idle -> s_TX_Start_Bit -> s_TX_Data_Bits (x8) ->
--                s_TX_Stop_Bit -> s_Cleanup -> s_Idle.
--                Chaque bit dure g_CLKS_PER_BIT périodes d'horloge, mesurées
--                par le compteur r_Clk_Count. La ligne est à '1' au repos.
--                L'octet est copié dans r_TX_Data au départ : l'entrée
--                i_TX_Byte peut ensuite changer sans perturber l'envoi.
--                Toutes les sorties sont des bascules (affectées sur front).
--                Dans le robot : i_Clk = 500 kHz et g_CLKS_PER_BIT = 52,
--                soit 104 µs par bit, 9615 bauds (9600 bauds à 0,16 % près),
--                1,04 ms par octet.
-- Entrées      : i_Clk     : horloge (500 kHz dans le robot).
--                i_TX_DV   : '1' pendant au moins un cycle quand l'émetteur
--                            est au repos : lance l'envoi de i_TX_Byte.
--                i_TX_Byte : octet à envoyer.
-- Sorties      : o_TX_Active : '1' pendant l'envoi (du bit de start à la fin
--                              du bit de stop) ; passe à '1' un cycle après
--                              la prise en compte de i_TX_DV.
--                o_TX_Serial : ligne série vers le PC (repos à '1').
--                o_TX_Done   : '1' en fin d'envoi. Attention : il reste à '1'
--                              pendant 2 cycles (fin du bit de stop puis
--                              s_Cleanup), et non 1 comme l'indiquait la
--                              note d'origine.
-- Génériques   : g_CLKS_PER_BIT : durée d'un bit en périodes de i_Clk,
--                = fréquence d'horloge / débit (500 000 / 9600 = 52 ici).
-- Remarques    : Pas d'entrée de reset (code fourni) : l'état initial vient
--                des valeurs de mise sous tension des signaux. o_TX_Serial
--                n'a pas de valeur initiale : sur la carte le registre
--                démarrerait à '0' et l'horloge 500 kHz est arrêtée pendant
--                le reset (1 ms), la ligne serait donc basse au démarrage et
--                le PC recevrait un faux octet. Corrigé sans toucher à ce
--                code : robot_sumo_top force la ligne à '1' pendant le reset
--                et les 2 premiers cycles d'horloge qui suivent.
--                Un reset (BP1) en cours de trame fige la trame : elle est
--                abîmée côté PC ; message_demarrage attend ensuite une trame
--                de silence avant de renvoyer le message.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity UART_TX is
  generic (
    g_CLKS_PER_BIT : integer := 115     -- durée d'un bit en cycles d'horloge (52 dans le robot)
    );
  port (
    i_Clk       : in  std_logic;                     -- horloge de cadencement
    i_TX_DV     : in  std_logic;                     -- demande d'envoi (Data Valid)
    i_TX_Byte   : in  std_logic_vector(7 downto 0);  -- octet à envoyer
    o_TX_Active : out std_logic;                     -- '1' pendant l'envoi d'une trame
    o_TX_Serial : out std_logic;                     -- ligne série TX vers le PC
    o_TX_Done   : out std_logic                      -- '1' en fin de trame (2 cycles)
    );
end UART_TX;


architecture RTL of UART_TX is

  -- États de la machine d'émission, dans l'ordre d'une trame 8N1.
  type t_SM_Main is (s_Idle, s_TX_Start_Bit, s_TX_Data_Bits,
                     s_TX_Stop_Bit, s_Cleanup);
  signal r_SM_Main : t_SM_Main := s_Idle;             -- état courant, au repos à la mise sous tension

  -- Compte les cycles d'horloge écoulés dans le bit en cours (0 à g_CLKS_PER_BIT-1).
  signal r_Clk_Count : integer range 0 to g_CLKS_PER_BIT-1 := 0;
  -- Numéro du bit de données en cours d'envoi (0 = poids faible, envoyé en premier).
  signal r_Bit_Index : integer range 0 to 7 := 0;  -- 8 Bits Total
  -- Copie de l'octet à envoyer, figée au départ de la trame.
  signal r_TX_Data   : std_logic_vector(7 downto 0) := (others => '0');
  -- Indicateur interne de fin de trame, recopié sur o_TX_Done.
  signal r_TX_Done   : std_logic := '0';

begin


  -- Machine à états d'émission : un seul processus synchrone, toutes les
  -- sorties sont mises à jour sur le front montant de i_Clk.
  p_UART_TX : process (i_Clk)
  begin
    if rising_edge(i_Clk) then

      case r_SM_Main is

        -- Repos : ligne à '1', compteurs remis à zéro. Une demande d'envoi
        -- (i_TX_DV = '1') mémorise l'octet et démarre la trame.
        when s_Idle =>
          o_TX_Active <= '0';
          o_TX_Serial <= '1';         -- Drive Line High for Idle
          r_TX_Done   <= '0';
          r_Clk_Count <= 0;
          r_Bit_Index <= 0;

          if i_TX_DV = '1' then
            r_TX_Data <= i_TX_Byte;   -- octet figé pour toute la durée de la trame
            r_SM_Main <= s_TX_Start_Bit;
          else
            r_SM_Main <= s_Idle;
          end if;

        -- Bit de start : la ligne passe à '0' pendant une durée de bit, ce
        -- qui signale au récepteur le début d'une trame.
        when s_TX_Start_Bit =>
          o_TX_Active <= '1';
          o_TX_Serial <= '0';

          -- Attente de la fin du bit de start (g_CLKS_PER_BIT cycles au total).
          if r_Clk_Count < g_CLKS_PER_BIT-1 then
            r_Clk_Count <= r_Clk_Count + 1;
            r_SM_Main   <= s_TX_Start_Bit;
          else
            r_Clk_Count <= 0;
            r_SM_Main   <= s_TX_Data_Bits;
          end if;

        -- Bits de données : chaque bit de l'octet est présenté sur la ligne
        -- pendant une durée de bit, du poids faible (bit 0) au poids fort.
        when s_TX_Data_Bits =>
          o_TX_Serial <= r_TX_Data(r_Bit_Index);

          if r_Clk_Count < g_CLKS_PER_BIT-1 then
            r_Clk_Count <= r_Clk_Count + 1;
            r_SM_Main   <= s_TX_Data_Bits;
          else
            r_Clk_Count <= 0;

            -- Bit suivant, ou passage au bit de stop après le bit 7.
            if r_Bit_Index < 7 then
              r_Bit_Index <= r_Bit_Index + 1;
              r_SM_Main   <= s_TX_Data_Bits;
            else
              r_Bit_Index <= 0;
              r_SM_Main   <= s_TX_Stop_Bit;
            end if;
          end if;

        -- Bit de stop : ligne à '1' pendant une durée de bit, puis fin de
        -- trame signalée par r_TX_Done.
        when s_TX_Stop_Bit =>
          o_TX_Serial <= '1';

          if r_Clk_Count < g_CLKS_PER_BIT-1 then
            r_Clk_Count <= r_Clk_Count + 1;
            r_SM_Main   <= s_TX_Stop_Bit;
          else
            r_TX_Done   <= '1';
            r_Clk_Count <= 0;
            r_SM_Main   <= s_Cleanup;
          end if;

        -- Cycle de transition vers le repos : o_TX_Active retombe à '0',
        -- o_TX_Done est maintenu un cycle de plus. L'émetteur accepte une
        -- nouvelle demande au cycle suivant (état s_Idle).
        when s_Cleanup =>
          o_TX_Active <= '0';
          r_TX_Done   <= '1';
          r_SM_Main   <= s_Idle;

        -- Sécurité : tout état non prévu ramène au repos.
        when others =>
          r_SM_Main <= s_Idle;

      end case;
    end if;
  end process p_UART_TX;

  -- Indicateur de fin de trame vers l'extérieur.
  o_TX_Done <= r_TX_Done;

end RTL;
