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

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;



entity samplingClock is
	port (
		conduit_end_new_signal : out std_logic;
		double_freq_new_signal : out std_logic;       
		clock_sink_1_clk       : in  std_logic := '0' -- 50MHz in
	);
end entity samplingClock;

architecture rtl of samplingClock is
	signal fullCount, halfCount: unsigned(31 downto 0) := "00000000000000000000000000000000";
	signal fullEnd, halfEnd: std_logic := '0';
	CONSTANT intOfZeros : unsigned := "00000000000000000000000000000000";
	CONSTANT tenHertz : unsigned := "0000000001001100010010110100000";
	CONSTANT fiveHertz : unsigned := "0000000010011000100101101000000";
	CONSTANT twoHertz : unsigned := "0000000101111101011110000100000";
	CONSTANT oneHerta : unsigned := "0000001011111010111100001000000";
begin
	
	fullClkDivide:process(clock_sink_1_clk) is
	begin
		if rising_edge(clock_sink_1_clk) then
			fullCount <= fullCount + 1;
		end if;
		
		if (fullCount = fiveHertz) then -- See constants in architecture for different frequencies
			fullCount <= intOfZeros;
			fullEnd <= NOT fullEnd;
		end if;
		conduit_end_new_signal <= fullEnd;
	end process fullClkDivide;

	halfClkDivide:process(clock_sink_1_clk) is
	begin
		if rising_edge(clock_sink_1_clk) then
			halfCount <= halfCount + 1;
		end if;
		
		if (halfCount = (fiveHertz/2)) then -- None of our frequencies are % 2 = 1 so odd/evens are not a problem
			halfCount <= intOfZeros;
			halfEnd <= NOT halfEnd;
		end if;
		double_freq_new_signal <= halfEnd;
	end process halfClkDivide;

end architecture rtl; -- of samplingClock