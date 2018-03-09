--------------------------------------------------------------------------------
--
--   FileName:         pwm.vhd
--   Dependencies:     none
--   Design Software:  Quartus II 64-bit Version 12.1 Build 177 SJ Full Version
--
--   HDL CODE IS PROVIDED "AS IS."  DIGI-KEY EXPRESSLY DISCLAIMS ANY
--   WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
--   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
--   PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL DIGI-KEY
--   BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
--   DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
--   PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
--   BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
--   ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
--
--   Version History
--   Version 1.0 8/1/2013 Scott Larson
--     Initial Public Release
--   Version 2.0 1/9/2015 Scott Larson
--     Transistion between duty cycles always starts at center of pulse to avoid
--     anomalies in pulse shapes
--    
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Modified By Brian Ofrim, Feb, 1 2018
-- Changes: generic constants
--          Input source changed to Avalon Slave
--         	Added a direction output
--          Simplified logic to just generate one pwm signal
--------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_unsigned.all;
USE ieee.std_logic_signed.all;
use IEEE.numeric_std.all;
ENTITY pwm IS
  GENERIC(
      sys_clk         : INTEGER := 50_000_000; --system clock frequency in Hz
      pwm_freq        : INTEGER := 50_000;    --PWM switching frequency in Hz
      bits_resolution : INTEGER := 8);         --bits of resolution setting the duty cycle
  PORT(
      clk       : IN  STD_LOGIC;                                    --system clock
      reset_n   : IN  STD_LOGIC;                                    --asynchronous reset                                 --latches in new duty cycle
		avalon_slave_write_n   : in  std_logic                     := '0';             --  avalon_slave.write
		avalon_slave_writedata : in  std_logic_vector(31 downto 0) := (others => '0'); --  duty
		dir       : OUT STD_LOGIC;
		pwm_temp  : OUT STD_LOGIC);
END pwm;

ARCHITECTURE logic OF pwm IS
  CONSTANT  period     :  INTEGER := sys_clk/pwm_freq;                      --number of clocks in one pwm period
  SIGNAL  count        :  INTEGER RANGE 0 TO period - 1;
  SIGNAL   half_duty_new  :  INTEGER RANGE 0 TO period/2 := 0;              --number of clocks in 1/2 duty cycle
  SIGNAL  half_duty    :  INTEGER RANGE 0 TO period/2;                     --array of half duty values (for each phase)
  SIGNAL  input_signed :  signed(3 downto 0) := (others => '0');
  SIGNAL  input_magnitude : signed(3 downto 0) := (others => '0');
BEGIN
  PROCESS(clk, reset_n, avalon_slave_write_n )
  BEGIN
    IF(reset_n = '0') THEN                                                 --asynchronous reset
      count <= 0;                                                --clear counter                                          --clear pwm inverse outputs
		pwm_temp <= '0';
	ELSIF(avalon_slave_write_n = '0') THEN
		input_signed <= signed(avalon_slave_writedata(3 downto 0));
		input_magnitude <= signed(abs(input_signed));
		half_duty_new <= to_integer(input_magnitude)*period/(2**bits_resolution)/2;   --determine clocks in 1/2 duty cycle
		if (input_signed > 0) then
			dir <= '1';
		else
			dir <= '0';
		end if;
    ELSIF(clk'EVENT AND clk = '1') THEN                                      --rising system clock edge   
		  IF(count = period - 1) THEN                       --end of period reached
          count <= 0;                                                         --reset counter
          half_duty <= half_duty_new;                                         --set most recent duty cycle value
        ELSE                                                                   --end of period not reached
          count <= count + 1;                                              --increment counter
        END IF;
        IF(count = half_duty) THEN                                       --phase's falling edge reached
          pwm_temp <= '0';
        ELSIF(count = period - half_duty) THEN                           --phase's rising edge reached
			 pwm_temp <= '1';
        END IF;
    END IF;
  END PROCESS;
  
END logic;
