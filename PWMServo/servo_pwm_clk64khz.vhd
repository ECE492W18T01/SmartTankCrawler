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

entity servo_pwm_clk64kHz is
    PORT(
        clk  : IN  STD_LOGIC;
        reset: IN  STD_LOGIC;
        pos  : IN  STD_LOGIC_VECTOR(6 downto 0);
        servo: OUT STD_LOGIC
    );
end servo_pwm_clk64kHz;

architecture Behavioral of servo_pwm_clk64kHz is
    COMPONENT clk64kHz
        PORT(
            clk    : in  STD_LOGIC;
            reset  : in  STD_LOGIC;
            clk_out: out STD_LOGIC
        );
    END COMPONENT;
    
    COMPONENT servo_pwm
        PORT (
            clk   : IN  STD_LOGIC;
            reset : IN  STD_LOGIC;
            pos   : IN  STD_LOGIC_VECTOR(6 downto 0);
            servo : OUT STD_LOGIC
        );
    END COMPONENT;
    
    signal clk_out : STD_LOGIC := '0';
begin
    clk64kHz_map: clk64kHz PORT MAP(
        clk, reset, clk_out
    );
    
    servo_pwm_map: servo_pwm PORT MAP(
        clk_out, reset, pos, servo
    );
end Behavioral;