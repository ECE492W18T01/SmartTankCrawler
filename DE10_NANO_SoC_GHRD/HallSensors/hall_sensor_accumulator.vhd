-- ECE 492 Wi 18 Team 1
-- January 30, 2018
-- Keith Mills

-- Stores rising and falling edges from an input signal edge_input
-- And loads the result onto a 16-bit output when the input
-- sampleClk (10-1Hz) goes high. Also includes a reset.

-- Modified February 23, 2018
-- Added integration with hall_debouncer
-- for cleaning up input signals.

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use ieee.VITAL_Primitives.all;

entity hall_sensor_accumulator is
	port ( -- Hall Sensor Input, Reset, system clock for debouncer Output From Internal Register
		edge_input   : in  std_logic := '0';
		reset	       : in  std_logic := '0';
		sysClk	    : in  std_logic := '0';
		sendValue    : out std_logic_vector(9 downto 0) := "0000000001"
	);
end entity hall_sensor_accumulator;

architecture rtl of hall_sensor_accumulator is
	
	CONSTANT tenBitsOne : unsigned := "0000000001";
	--type state_type is (reading, clearing); -- Read lines, reset the register, reset register to +1 normal reset value.
	--signal pres_state: state_type;
	signal cleanEdge : std_logic := '0';
	signal registerPos, registerNeg : unsigned(9 downto 0) := tenBitsOne; -- Set to 1 so as to avoid divide by zero.

	component hall_debouncer is
		port (
			clk	  :	in std_logic;
			reset	  :	in std_logic;
			hallRough :	in std_logic;
			hallClean :	out std_logic := '0'
		);
	end component hall_debouncer;
begin

	-- Feed input signal through debouncer
	cleanSignal: hall_debouncer
		port map(
			clk => sysClk,
			reset => reset,
			hallRough => edge_input,
			hallClean => cleanEdge
		);
		
	accum:process(cleanEdge) is
	begin
		
		if rising_edge(cleanEdge) then
			registerPos <= registerPos + 1;
		elsif falling_edge(cleanEdge) then
			registerNeg <= registerNeg + 1;
		end if;
	end process accum;
	
	sendValue <= std_logic_vector(registerPos + registerNeg);
	
end architecture rtl; -- hall_sensor_accumulator