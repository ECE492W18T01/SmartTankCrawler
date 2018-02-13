-- proximity_tb.vhd

-- ECE 492 Wi18
-- Feb 9, 2018
-- Brian OFrim

-- Testbench for proximity.vhd


library ieee;
use ieee.std_logic_1164.all;

entity proximity_tb is
end proximity_tb;

architecture behave of proximity_tb is
	signal clock:    std_logic := '0';
	constant CLOCK_period : time := 20ns;
	signal test_pw_signal:std_logic := '0';
	signal proximity_inches_test:  STD_LOGIC_VECTOR(31 downto 0):= (others => '0');

	component  proximity is 
		port(
      		clk       : IN  STD_LOGIC;                                    --system clock
      		reset_n   : IN  STD_LOGIC;                                    --asynchronous reset                                 --latches in new duty cycle
		pw_signal : IN  STD_LOGIC;
		avalon_slave_read_n : in  std_logic;
		proximity_inches_avalon_readdata: OUT STD_LOGIC_VECTOR(31 downto 0));
	end component proximity;

begin

	PROXYIMITY_INST : proximity
		port map(
			clk => clock,
			pw_signal => test_pw_signal,
			reset_n => '1',
			avalon_slave_read_n => '1',
			proximity_inches_avalon_readdata => proximity_inches_test
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
		wait for 1ms;
		test_pw_signal <= '1';
		wait for 10ms; 
		test_pw_signal <= '0';
		wait for 38ms;

		wait for 1ms;
		test_pw_signal <= '1';
		wait for 10.1ms; 
		test_pw_signal <= '0';
		wait for 37.9ms;


		wait for 1ms;
		test_pw_signal <= '1';
		wait for 20ms;
		test_pw_signal <= '0';
		wait for 28ms;

		wait for 1ms;
		test_pw_signal <= '1';
		wait for 30ms;
		test_pw_signal <= '0';
		wait for 18ms;

	end process sim_proc;
END;