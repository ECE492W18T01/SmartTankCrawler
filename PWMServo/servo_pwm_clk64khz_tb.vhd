--------------------------------------------------------------------------------
--
--   FileName:         servo_pwm_clk64khz_tb
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


LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
 
ENTITY servo_pwm_clk64kHz_tb IS
END servo_pwm_clk64kHz_tb;
 
ARCHITECTURE behavior OF servo_pwm_clk64kHz_tb IS
    -- Unit under test.
    COMPONENT servo_pwm_clk64kHz
        PORT(
            clk   : IN  std_logic;
            reset : IN  std_logic;
            pos   : IN  std_logic_vector(6 downto 0);
            servo : OUT std_logic
        );
    END COMPONENT;

    -- Inputs.
    signal clk  : std_logic := '0';
    signal reset: std_logic := '0';
    signal pos  : std_logic_vector(6 downto 0) := (others => '0');
    -- Outputs.
    signal servo : std_logic;
    -- Clock definition.
    constant clk_period : time := 10 ns;
BEGIN
    -- Instance of the unit under test.
    uut: servo_pwm_clk64kHz PORT MAP (
        clk => clk,
        reset => reset,
        pos => pos,
        servo => servo
    );

   -- Definition of the clock process.
   clk_process :process begin
        clk <= '0';
        wait for clk_period/2;
        clk <= '1';
        wait for clk_period/2;
   end process;
 
    -- Stimuli process.
    stimuli: process begin
        reset <= '1';
        wait for 50 ns;
        reset <= '0';
        wait for 50 ns;
        pos <= "0000000";
        wait for 20 ms;
        pos <= "0101000";
        wait for 20 ms;
        pos <= "1010000";
        wait for 20 ms;
        pos <= "1111000";
        wait for 20 ms;
        pos <= "1111111";
        wait;
    end process;
END;