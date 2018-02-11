-- samplingClock_tb.vhd

-- ECE 492 Wi18
-- Jan 30, 2018
-- Keith Mills

-- Testbench for samplingClock.vhd

library ieee;
use ieee.std_logic_1164.all;

entity samplingClock_tb is
end samplingClock_tb;

architecture behave of samplingClock_tb is
	signal fullOut, halfOut: std_logic := '0';
	signal clk:    std_logic := '0';
	constant CLOCK_period : time := 20ns;

	component samplingClock is
		port (
			conduit_end_new_signal : out std_logic;
			double_freq_new_signal : out std_logic;       
			clock_sink_1_clk       : in  std_logic := '0'
		);
	end component samplingClock;

begin

	STL_INST : samplingClock
		port map (
			conduit_end_new_signal => fullOut,
			double_freq_new_signal => halfOut,
			clock_sink_1_clk       => clk
		);

	CLOCK_process :process
	begin
		clk <= '0';
		wait for CLOCK_period/2;
		clk <= '1';
		wait for CLOCK_period/2;
	end process;

	stim_proc: process
	begin
		wait for CLOCK_period * 100000000;
	end process;
END;