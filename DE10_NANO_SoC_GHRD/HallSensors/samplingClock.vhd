-- samplingClock.vhd

-- ECE 492 Wi18
-- Jan 29, 2018
-- Keith Mills

-- Basically a clock divider from 50MHz
-- Downto roughly 5Hz or 2Hz, something very
-- small for the sampling purpose of our project
-- since we cannot use a PLL for such a low frequency.
-- See the bottom of the document for register values
-- to output different frequencies

-- Feb 10 Edit
-- I have chosen to add an additional output 
-- clock that runs at twice the frequency as the 
-- original sampling rate.
-- This will be used in the comparator to 
-- send updates to the CPU faster.

-- Feb 24 Edit
-- Added the sample_spike_signal_00 which is a short impulse
-- timed with the original clock, used to activate the load
-- feature of the Hall Sensor Accumulators. 

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;



entity samplingClock is
	port (
		sample_impulse_signal  : out std_logic;
		normal_freq_new_signal : out std_logic;
		double_freq_new_signal : out std_logic;       
		reset : in std_logic := '0';
		clock_sink_1_clk       : in  std_logic := '0' -- 50MHz in
	);
end entity samplingClock;

architecture rtl of samplingClock is
	CONSTANT intOfZeros : unsigned := "00000000000000000000000000000000"; -- Array of Zeros
	CONSTANT intOne     : unsigned := "00000000000000000000000000000001"; -- Array of one.

	CONSTANT tenHertz  : unsigned := "00000000001001100010010110100000";  -- Time values
	CONSTANT fiveHertz : unsigned := "00000000010011000100101101000000";
	CONSTANT twoHertz  : unsigned := "00000000101111101011110000100000"; -- Choosing this since we don't have access to the input shaft
	CONSTANT oneHertz  : unsigned := "00000001011111010111100001000000"; -- of our electric motors

	signal fullCount, halfCount: unsigned(31 downto 0) := intOfZeros; -- Initialize clock counters to zero.
	signal sampleCount: unsigned(31 downto 0) := (twoHertz - 400); -- Initalize impulse counter to half way so it syncs with the full clock.
	signal sampleEnd, fullEnd, halfEnd: std_logic := '0';
begin
	
	sampleImpulse:process(clock_sink_1_clk) is
	begin
		if rising_edge(clock_sink_1_clk) then
			sampleCount <= sampleCount + 1;
		end if;
		
		if (sampleCount = (twoHertz*2)-200) AND sampleEnd = '0' then -- See constants in architecture for different frequencies
			sampleCount <= intOfZeros;
			sampleEnd <= '1';
		elsif (sampleCount = 200) AND sampleEnd = '1' then
			sampleCount <= intOfZeros;
			sampleEnd <= '0';
		end if;
		sample_impulse_signal <= sampleEnd;
	end process sampleImpulse;

	fullClkDivide:process(clock_sink_1_clk) is
	begin
		if rising_edge(clock_sink_1_clk) then
			fullCount <= fullCount + 1;
		end if;
		
		if (fullCount = twoHertz) then -- See constants in architecture for different frequencies
			fullCount <= intOfZeros;
			fullEnd <= NOT fullEnd;
		end if;
		normal_freq_new_signal <= fullEnd;
	end process fullClkDivide;

	halfClkDivide:process(clock_sink_1_clk) is
	begin
		if rising_edge(clock_sink_1_clk) then
			halfCount <= halfCount + 1;
		end if;
		
		if (halfCount = (twoHertz/2)) then -- None of our frequencies are % 2 = 1 so odd/evens are not a problem
			halfCount <= intOfZeros;
			halfEnd <= NOT halfEnd;
		end if;
		double_freq_new_signal <= halfEnd;
	end process halfClkDivide;

end architecture rtl; -- of samplingClock