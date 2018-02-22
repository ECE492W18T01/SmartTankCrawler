--------------------------------------------------------------------------------
--
--   FileName:         servo_pwm
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
--    Modified by Keith Mills (kgmills, Ualberta, ECE 492 Wi19 Team 1) on 2/22/18
--    Changed input data length to match our need for an Altera DE10-Nano board
--    Slight modification to output generation code. 
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity servo_pwm is
    -- Modified ports for compliance with Avalon Memory-Mapped Slave Interface
    PORT (
        clk   : IN  STD_LOGIC;
        reset : IN  STD_LOGIC;
        servo   : IN  STD_LOGIC_VECTOR(7 downto 0);
	writeIn : IN  STD_LOGIC;
        position : OUT STD_LOGIC
    );
end servo_pwm;

architecture Behavioral of servo_pwm is
    -- Counter, from 0 to 1279.
    signal cnt : unsigned(10 downto 0);
    -- Temporal signal used to generate the PWM pulse.
    -- Bus length changed from 7 to 8 as we must write a width of 8 in software.
    signal pwmi: unsigned(7 downto 0);
	 -- Middle-man line for output.
	 signal intermediate : std_logic := '0';
begin
    -- Minimum value should be 0.5ms.
    -- Kgmills chose to comment out the 32, it still works on our Servos.
    pwmi <= unsigned(servo);-- + 32;
    -- Counter process, from 0 to 1279.
    counter: process (reset, clk) begin
        if (reset = '1') then
            cnt <= (others => '0');
        elsif rising_edge(clk) then
            if (cnt = 1279) then
                cnt <= (others => '0');
            else
                cnt <= cnt + 1;
            end if;
        end if;
    end process;
    -- Output signal for the servomotor.
    -- This did not work the DE10-Nano.
    --position <= '1' when (cnt < pwmi) else '0';

    -- This on the other hand, did. 
	 setOutput: process(cnt)
	 begin
			if (cnt < pwmi) then
				intermediate <= '1';
			else
				intermediate <= '0';
			end if;
			position <= intermediate;
	 end process setOutput;
end Behavioral;