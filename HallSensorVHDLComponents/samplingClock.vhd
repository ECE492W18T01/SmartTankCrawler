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

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity samplingClock is
	port (
		conduit_end_new_signal : out std_logic;       
		clock_sink_1_clk       : in  std_logic := '0' -- 50MHz in
	);
end entity samplingClock;

architecture rtl of samplingClock is
	signal counter: unsigned(31 downto 0) := "00000000000000000000000000000000";
	signal newEnd: std_logic := '0';
begin
	
	clock_divide:process(clock_sink_1_clk) is
	begin
		if rising_edge(clock_sink_1_clk) then
			counter <= counter + 1;
		end if;
		
		if (counter = "0000000010011000100101101000000") then -- See bottom of document for other values
			counter <= "00000000000000000000000000000000";
			newEnd <= NOT newEnd;
		end if;
		conduit_end_new_signal <= newEnd;
	end process clock_divide;

end architecture rtl; -- of samplingClock

-- Values for different output frequencies
-- 10Hz-> "0000000001001100010010110100000"
-- 5Hz -> "0000000010011000100101101000000"
-- 2Hz -> "0000000101111101011110000100000"
-- 1Hz -> "0000001011111010111100001000000"