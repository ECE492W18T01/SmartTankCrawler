--------------------------------------------------------------------------------
--
--   FileName:         clk64kHz
--   Dependencies:     none
--
--   GNU LESSER GENERAL PUBLIC LICENSE
--   Version 3, 29 June 2007
--
--   Copyright (C) 2007 Free Software Foundation, Inc. <http://fsf.org/>
--   Article on Servo motor control with PWM and VHDL   
--   https://www.codeproject.com/Articles/513169/Servomotor-Control-with-PWM-and-VHDL
--
--   Version History
--   Version 1.0 12/20/2012 Carl A. Ramos
--     Initial Public Release
--     generate a pulse width modulation signal for a servo motor control with vhdl.
-- 
--    
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
 
entity clk64kHz is
    Port (
        clk    : in  STD_LOGIC;
        reset  : in  STD_LOGIC;
        clk_out: out STD_LOGIC
    );
end clk64kHz;
 
architecture Behavioral of clk64kHz is
    signal temporal: STD_LOGIC;
    signal counter : integer range 0 to 780 := 0;
begin
    freq_divider: process (reset, clk) begin
        if (reset = '1') then
            temporal <= '0';
            counter  <= 0;
        elsif rising_edge(clk) then
            if (counter = 780) then
                temporal <= NOT(temporal);
                counter  <= 0;
            else
                counter <= counter + 1;
            end if;
        end if;
    end process;
 
    clk_out <= temporal;
end Behavioral;