
-- hall_sensor_accumulator_tb.vhd

-- ECE 492 Wi18
-- Feb 1, 2018
-- Keith Mills

-- Testbench for hall_sensor_accumulator.vhd
-- Dependant on the custom component samplingClock.vhd
-- when using the architecture 'withClock'

library ieee;
use ieee.std_logic_1164.all;

entity hall_sensor_accumulator_tb is
end hall_sensor_accumulator_tb;

architecture withClock of hall_sensor_accumulator_tb is
	signal sClkOut: std_logic := '0';
	signal clk:    std_logic := '0';
	constant CLOCK_period : time := 20ns;
	
	signal edge: std_logic := '0';
	signal rSet: std_logic := '0';
	signal accum: std_logic_vector(9 downto 0) := "0000000000";
	constant edge_period : time := 5ms;

	component samplingClock is
		port (
			conduit_end_new_signal : out std_logic;        --  conduit_end.export
			double_freq_new_signal : out std_logic;        -- Unused for this tool.
			clock_sink_1_clk       : in  std_logic := '0'  -- clock_sink_1.clk
			);
	end component samplingClock;

	component hall_sensor_accumulator is
		port (
			edge_input : in  std_logic := '0';
			reset		  : in  std_logic := '0';
			sampleClk  : in  std_logic := '0';
			output     : out std_logic_vector(9 downto 0)
		);
	end component hall_sensor_accumulator;

begin

	STL_INST : samplingClock
		port map (
			conduit_end_new_signal => sClkOut,
			double_freq_new_signal => open,
			clock_sink_1_clk       => clk
		);

	OTR_INST : hall_sensor_accumulator
		port map (
			edge_input => edge,
			reset      => rSet,
			sampleClk  => sCLKOut,
			output     => accum
		);	

	EDGE_process :process -- Increase the edge speed so as to get different numbers. 
	variable divide: Integer:= 2;
	variable count:  Integer:= 0;
	begin
		edge <= '0';
		wait for edge_period/divide;
		edge <= '1';
		wait for edge_period/divide;
		count := count + 1;
		if count = 40 then
			divide := divide + 1;
			count := 0;
		end if;
	end process EDGE_process;

	CLOCK_process :process
	begin
		clk <= '0';
		wait for CLOCK_period/2;
		clk <= '1';
		wait for CLOCK_period/2;
	end process CLOCK_process;

	stim_proc: process
	begin
		wait for CLOCK_period * 100000000;
	end process;
END;

architecture withoutClock of hall_sensor_accumulator_tb is
	signal sClkOut: std_logic := '0';
	constant CLOCK_period : time := 200ms;
	
	signal edge: std_logic := '0';
	signal rSet: std_logic := '0';
	signal accum: std_logic_vector(9 downto 0) := "0000000000";
	constant edge_period : time := 5ms;

	component hall_sensor_accumulator is
		port (
			edge_input : in  std_logic := '0';
			reset		  : in  std_logic := '0';
			sampleClk  : in  std_logic := '0';
			output     : out std_logic_vector(9 downto 0)
		);
	end component hall_sensor_accumulator;

begin

	OTR_INST : hall_sensor_accumulator
		port map (
			edge_input => edge,
			reset      => rSet,
			sampleClk  => sCLKOut,
			output     => accum
		);	

	EDGE_process :process -- Increase the edge speed so as to get different numbers. 
	variable divide: Integer:= 2;
	variable count:  Integer:= 0;
	begin
		edge <= '0';
		wait for edge_period/divide;
		edge <= '1';
		wait for edge_period/divide;
		count := count + 1;
		if count = 40 then
			divide := divide + 1;
			count := 0;
		end if;
	end process EDGE_process;

	CLOCK_process :process
	begin
		sCLKOut <= '0';
		wait for CLOCK_period/2;
		sCLKOut <= '1';
		wait for CLOCK_period/2;
	end process CLOCK_process;

	stim_proc: process
	begin
		wait for CLOCK_period * 100000000;
	end process;
END;