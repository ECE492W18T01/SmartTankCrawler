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
--          Input sourcperiod : INTEGER := sys_clk/pwm_freq;e changed to Avalon Slave
--         	Added a direction output
--          Simplified logic to just generate one pwm signal
--------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_unsigned.all;
USE ieee.std_logic_signed.all;
use IEEE.numeric_std.all;

-- direction information
-- pwm_signal=1, dir_1=1, dir_2=0 -> forward
-- pwm_signal=1, dir_1=0, dir_2=1 -> reverse
-- pwm_signal=1, dir_1=0, dir_2=0 -> brake
-- pwm_signal=1, dir_1=1, dir_2=1 -> brake
-- pwm_signal=0, dir_1=X, dir_2=X -> off

ENTITY pwm IS
	GENERIC(
		--system clock frequency in Hz
		sys_clk : INTEGER := 50_000_000;
		--PWM switching frequency in Hz
		pwm_freq : INTEGER := 10_000;
		--bits of resolution setting the duty cycle
      		bits_resolution : INTEGER := 8);         
	PORT(
		--system clock
		clk : IN  STD_LOGIC;
		--asynchronous reset
		reset_n : IN  STD_LOGIC;
		-- indicator for when the is a new motor value
		avalon_slave_write_n : in  std_logic := '0';
		-- input motor value
		avalon_slave_writedata : in std_logic_vector(7 downto 0) := (others => '0');
		-- direction outputs
		dir_1 : OUT STD_LOGIC;
		dir_2 : OUT STD_LOGIC;
		pwm_signal : OUT STD_LOGIC);
END pwm;


ARCHITECTURE logic OF pwm IS
	--number of clocks in one pwm period
	CONSTANT period : INTEGER := sys_clk/pwm_freq;
	--max duty cycle value 
	CONSTANT max_pwm: INTEGER := 2**(bits_resolution -1) - 1;
	-- duty cycle multiplier
	CONSTANT duty_cycle_multiplier: INTEGER := period/max_pwm;
	-- clock cycle count
	SIGNAL count: INTEGER RANGE 0 TO period - 1;
	--number of clocks in 1/2 duty cycle
	SIGNAL half_duty_new : INTEGER RANGE 0 TO period := 0;
	-- count value for which the pwm switches to low
	SIGNAL half_duty : INTEGER RANGE 0 TO period;
	SIGNAL input_signed : signed(7 downto 0) := (others => '0');
	SIGNAL input_magnitude : signed(7 downto 0) := (others => '0');
BEGIN
	PROCESS(clk, reset_n, avalon_slave_write_n )
	BEGIN
	IF(reset_n = '0') THEN
		-- reset control values
		count <= 0;
		pwm_signal <= '0';
		dir_1 <= '0';
		dir_2 <= '0'; 
	-- new data avaliable
	ELSIF(avalon_slave_write_n = '0') THEN
		input_signed <= signed(avalon_slave_writedata(7 downto 0));
		input_magnitude <= signed(abs(input_signed));
		--determine clocks in 1/2 duty cycle
		half_duty_new <= to_integer(input_magnitude)*duty_cycle_multiplier;
		-- forward
		if (input_signed > 0) then
			dir_1 <= '1';
			dir_2 <= '0';
		else
		-- reverse
			dir_1 <= '0';
			dir_2 <= '1';
		end if;
	--rising system clock edge 
	ELSIF(clk'EVENT AND clk = '1') THEN
		--end of period reached
		IF(count = period - 1) THEN
			--reset counter
			count <= 0;
			--set most recent duty cycle value
			half_duty <= half_duty_new;
		--end of period not reached
		ELSE
			--increment counter		
			count <= count + 1;
		END IF;
		--phase's falling edge reached
		IF(count <= half_duty) THEN
			pwm_signal <= '1';
		--phase's rising edge reached
		ELSE
			pwm_signal <= '0';
		END IF;
	END IF;
	END PROCESS;
END logic;

