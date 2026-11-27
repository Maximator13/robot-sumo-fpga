--------------------------------------------------------------------------------
-- Fichier      : UART_RX.vhd
-- Entité       : UART_RX
-- Projet       : Robot SUMO, DE0-Nano (Cyclone IV EP4CE22F17C6N)
-- Origine      : Code fourni par l'enseignant (cahier des charges §5), issu de
--                https://www.nandland.com (module UART). La logique n'est PAS
--                modifiée : seuls les commentaires et la mise en page ont été
--                retouchés.
-- Description  : Récepteur série asynchrone (UART) : reçoit les octets
--                envoyés par le PC via le module USB/série FT232R, au format
--                8N1 (1 bit de start, 8 bits de données poids faible en
--                premier, pas de parité, 1 bit de stop). Dans le robot il
--                reçoit les commandes 'c' (combat) et 's' (suiveur de ligne).
-- Principe     : 1) La ligne RX, asynchrone de l'horloge du FPGA, traverse
--                   deux bascules (p_SAMPLE) pour éviter la métastabilité.
--                2) Une machine à états (p_UART_RX) détecte le passage à '0'
--                   de la ligne (début du bit de start), puis vérifie la
--                   ligne g_CLKS_PER_BIT/2 = 26 cycles plus tard, soit
--                   environ au milieu du bit de start (à 2 ou 3 cycles près,
--                   dus à la synchronisation). Si elle est revenue à '1',
--                   c'était un parasite plus court qu'un demi-bit : il est
--                   rejeté.
--                3) Ensuite elle échantillonne chaque bit de données une durée
--                   de bit (g_CLKS_PER_BIT cycles) plus tard, donc près du
--                   milieu de chaque bit, ce qui tolère un petit écart de
--                   débit entre PC et robot (testé à +/- 2 %).
--                4) Une durée de bit après l'échantillon du bit 7, c'est-à-
--                   dire au MILIEU du bit de stop, o_RX_DV passe à '1'
--                   pendant un seul cycle : o_RX_Byte contient alors l'octet
--                   reçu. Le récepteur est de retour au repos 2 cycles plus
--                   tard, donc prêt pour un bit de start qui suivrait
--                   immédiatement le stop.
--                Le bit de stop n'est pas vérifié (pas de détection d'erreur
--                de trame). Dans le robot : i_Clk = 500 kHz et
--                g_CLKS_PER_BIT = 52 (104 µs par bit, soit 9615 bauds :
--                9600 bauds à 0,16 % près).
-- Entrées      : i_Clk       : horloge (500 kHz dans le robot).
--                i_RX_Serial : ligne série venant du PC (repos à '1').
-- Sorties      : o_RX_DV   : '1' pendant un cycle quand un octet est reçu.
--                o_RX_Byte : octet reçu. Il n'est valide QUE pendant
--                            l'impulsion o_RX_DV : il est réécrit bit par bit
--                            dès la trame suivante (à lire sur o_RX_DV = '1').
-- Génériques   : g_CLKS_PER_BIT : durée d'un bit en périodes de i_Clk,
--                = fréquence d'horloge / débit (500 000 / 9600 = 52 ici).
-- Remarques    : Pas d'entrée de reset (code fourni) : l'état initial vient
--                des valeurs de mise sous tension des signaux.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;

entity UART_RX is
  generic (
    g_CLKS_PER_BIT : integer := 115     -- durée d'un bit en cycles d'horloge (52 dans le robot)
    );
  port (
    i_Clk       : in  std_logic;                     -- horloge de cadencement
    i_RX_Serial : in  std_logic;                     -- ligne série RX venant du PC
    o_RX_DV     : out std_logic;                     -- impulsion "octet reçu" (Data Valid)
    o_RX_Byte   : out std_logic_vector(7 downto 0)   -- octet reçu
    );
end UART_RX;


architecture rtl of UART_RX is

  -- États de la machine de réception, dans l'ordre d'une trame 8N1.
  type t_SM_Main is (s_Idle, s_RX_Start_Bit, s_RX_Data_Bits,
                     s_RX_Stop_Bit, s_Cleanup);
  signal r_SM_Main : t_SM_Main := s_Idle;             -- état courant, au repos à la mise sous tension

  -- Les deux bascules de synchronisation de la ligne RX.
  signal r_RX_Data_R : std_logic := '0';              -- 1re bascule (peut être métastable)
  signal r_RX_Data   : std_logic := '0';              -- 2e bascule : niveau RX utilisable

  -- Compte les cycles d'horloge écoulés dans le bit en cours.
  signal r_Clk_Count : integer range 0 to g_CLKS_PER_BIT-1 := 0;
  -- Numéro du bit de données en cours de réception (0 = poids faible, reçu en premier).
  signal r_Bit_Index : integer range 0 to 7 := 0;  -- 8 Bits Total
  -- Octet en cours d'assemblage puis octet reçu.
  signal r_RX_Byte   : std_logic_vector(7 downto 0) := (others => '0');
  -- Impulsion "octet reçu", recopiée sur o_RX_DV.
  signal r_RX_DV     : std_logic := '0';

begin

  -- Purpose: Double-register the incoming data.
  -- This allows it to be used in the UART RX Clock Domain.
  -- (It removes problems caused by metastabiliy)
  -- Double bascule de synchronisation : la ligne RX change à des instants
  -- quelconques par rapport à i_Clk ; on n'utilise que la sortie de la
  -- seconde bascule (r_RX_Data), qui a eu un cycle complet pour se stabiliser.
  p_SAMPLE : process (i_Clk)
  begin
    if rising_edge(i_Clk) then
      r_RX_Data_R <= i_RX_Serial;
      r_RX_Data   <= r_RX_Data_R;
    end if;
  end process p_SAMPLE;

  -- Purpose: Control RX state machine
  -- Machine à états de réception (un seul processus synchrone).
  p_UART_RX : process (i_Clk)
  begin
    if rising_edge(i_Clk) then
      case r_SM_Main is

        -- Repos : on attend que la ligne passe à '0' (début du bit de start).
        when s_Idle =>
          r_RX_DV     <= '0';
          r_Clk_Count <= 0;
          r_Bit_Index <= 0;

          if r_RX_Data = '0' then       -- Start bit detected
            r_SM_Main <= s_RX_Start_Bit;
          else
            r_SM_Main <= s_Idle;
          end if;

        -- Check middle of start bit to make sure it's still low
        -- Bit de start : on compte jusqu'au milieu du bit. S'il est encore
        -- à '0', c'est un vrai début de trame et le compteur repart de 0
        -- (les échantillons suivants tomberont au milieu de chaque bit).
        -- Sinon c'était un parasite : retour au repos.
        when s_RX_Start_Bit =>
          if r_Clk_Count = (g_CLKS_PER_BIT-1)/2 then
            if r_RX_Data = '0' then
              r_Clk_Count <= 0;  -- reset counter since we found the middle
              r_SM_Main   <= s_RX_Data_Bits;
            else
              r_SM_Main   <= s_Idle;
            end if;
          else
            r_Clk_Count <= r_Clk_Count + 1;
            r_SM_Main   <= s_RX_Start_Bit;
          end if;

        -- Wait g_CLKS_PER_BIT-1 clock cycles to sample serial data
        -- (note d'origine imprécise : le compteur va de 0 à g_CLKS_PER_BIT-1,
        -- soit g_CLKS_PER_BIT cycles = une durée de bit entre deux échantillons)
        -- Bits de données : une durée de bit après le milieu du bit
        -- précédent, on est au milieu du bit suivant ; on le range à sa
        -- place dans l'octet (poids faible d'abord).
        when s_RX_Data_Bits =>
          if r_Clk_Count < g_CLKS_PER_BIT-1 then
            r_Clk_Count <= r_Clk_Count + 1;
            r_SM_Main   <= s_RX_Data_Bits;
          else
            r_Clk_Count            <= 0;
            r_RX_Byte(r_Bit_Index) <= r_RX_Data;

            -- Check if we have sent out all bits
            -- (note d'origine recopiée de l'émetteur : ici on vérifie si les
            -- 8 bits ont été REÇUS)
            -- Bit suivant, ou passage au bit de stop après le bit 7.
            if r_Bit_Index < 7 then
              r_Bit_Index <= r_Bit_Index + 1;
              r_SM_Main   <= s_RX_Data_Bits;
            else
              r_Bit_Index <= 0;
              r_SM_Main   <= s_RX_Stop_Bit;
            end if;
          end if;

        -- Receive Stop bit. Stop bit = 1
        -- Bit de stop : une durée de bit après l'échantillon du bit 7, on
        -- est au milieu du bit de stop ; sa valeur n'est pas contrôlée et
        -- l'octet reçu est signalé dès cet instant (o_RX_DV à '1').
        when s_RX_Stop_Bit =>
          -- Wait g_CLKS_PER_BIT-1 clock cycles for Stop bit to finish
          -- (note d'origine inexacte : l'attente dure g_CLKS_PER_BIT cycles et
          -- se termine au MILIEU du bit de stop, pas à sa fin)
          if r_Clk_Count < g_CLKS_PER_BIT-1 then
            r_Clk_Count <= r_Clk_Count + 1;
            r_SM_Main   <= s_RX_Stop_Bit;
          else
            r_RX_DV     <= '1';
            r_Clk_Count <= 0;
            r_SM_Main   <= s_Cleanup;
          end if;

        -- Stay here 1 clock
        -- Cycle de transition : o_RX_DV retombe (impulsion d'un seul cycle)
        -- et retour au repos, prêt pour l'octet suivant.
        when s_Cleanup =>
          r_SM_Main <= s_Idle;
          r_RX_DV   <= '0';

        -- Sécurité : tout état non prévu ramène au repos.
        when others =>
          r_SM_Main <= s_Idle;

      end case;
    end if;
  end process p_UART_RX;

  -- Sorties vers l'extérieur.
  o_RX_DV   <= r_RX_DV;
  o_RX_Byte <= r_RX_Byte;

end rtl;
