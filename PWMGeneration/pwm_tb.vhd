
-- proximity_tb.vhd

-- ECE 492 Wi18
-- Mar 5, 2018
-- Brian Ofrim

-- Testbench for pwm.vhd


library ieee;
use ieee.std_logic_1164.all;
USE ieee.std_logic_signed.all;
use IEEE.numeric_std.all;

entity pwm_tb is
end pwm_tb;

architecture behave of pwm_tb is
	signal clock: std_logic := '0';
	constant CLOCK_period : time := 20ns;
	signal test_pw_signal:std_logic := '0';
	signal avalon_slave_writedata_test : std_logic_vector(7 downto 0) := (others => '0');
	signal avalon_slave_write_n_test: std_logic := '1';
	signal dir_1_test: std_logic := '0';
	signal dir_2_test: std_logic := '0';

	signal dutyCycle1 : INTEGER := 50;
	signal dutyCycle2: INTEGER := -120;
	signal dutyCycle3: INTEGER := 10;

	component  pwm is 
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
		pwm_temp : OUT STD_LOGIC);
	end component pwm;
begin

	PWM_INST: pwm
	port map(
		clk => clock,
		reset_n => '1',
		avalon_slave_write_n => avalon_slave_write_n_test,
		avalon_slave_writedata => avalon_slave_writedata_test,
		dir_1 => dir_1_test,
		dir_2 => dir_2_test,
		pwm_temp => test_pw_signal
	);
	CLOCK_process :process
	begin
		clock <= '1';
		wait for CLOCK_period/2;
		clock <= '0';
		wait for CLOCK_period/2;
	end process CLOCK_process;

	sim_proc: process
	begin
		avalon_slave_write_n_test <= '1';
		wait for 1ms;
		avalon_slave_writedata_test <= std_logic_vector(to_signed(dutyCycle1, avalon_slave_writedata_test'length));
		wait for 1ms;
		avalon_slave_write_n_test <= '0';
		wait for 1ms;
		avalon_slave_write_n_test <= '1';
		wait for 20ms;
		avalon_slave_writedata_test <= std_logic_vector(to_signed(dutyCycle2, avalon_slave_writedata_test'length));
		wait for 1ms;
		avalon_slave_write_n_test <= '0';
		wait for 1ms;
		avalon_slave_write_n_test <= '1';
		wait for 20ms;
		avalon_slave_writedata_test <= std_logic_vector(to_signed(dutyCycle3, avalon_slave_writedata_test'length));
		wait for 1ms;
		avalon_slave_write_n_test <= '0';
		wait for 1ms;
		avalon_slave_write_n_test <= '1';
		wait for 20ms;
	end process sim_proc;
END;