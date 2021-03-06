-- hall_sensor_accumulator.vhd

-- ECE 492 Wi 18 Team 1
-- January 30, 2018
-- Keith Mills

-- Stores rising and falling edges from an input signal edge_input
-- And loads the result onto a 16-bit output when the input
-- sampleClk (10-1Hz) goes high. Also includes a reset.

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity hall_sensor_accumulator is
	port ( -- Hall Sensor Input, Reset, 1-10Hz Clock, Output From Internal Register
		edge_input : in  std_logic := '0';
		reset	   : in  std_logic := '0';
		sampleClk  : in  std_logic := '0';
		output     : out std_logic_vector(9 downto 0)
	);
end entity hall_sensor_accumulator;

architecture rtl of hall_sensor_accumulator is
	type state_type is (reading, clearing, readBoth); -- Read lines, reset the register, reset register to +1 normal reset value.
	signal pres_state: state_type;
	signal register8: unsigned(9 downto 0) := "0000000001";
begin
	run:process(edge_input, sampleClk, reset) is
	begin
		-- The 'just in case' the sampling clock and edge input happen simultaneously, reset to +2 instead of +1
		if (rising_edge(sampleClk) or rising_edge(reset)) and edge_input'event then
			output <= std_logic_vector(register8);
			pres_state <= readBoth;
		elsif rising_edge(sampleClk) or rising_edge(reset) then
			output <= std_logic_vector(register8);
			pres_state <= clearing;
		elsif rising_edge(edge_input) or falling_edge(edge_input) then
			case pres_state is
				when reading =>
					register8 <= register8 + 1; 
				when clearing => 		    
					register8 <= "0000000001"; -- Set it to one, as it will be used in division operations later
					pres_state <= reading;   -- So we can avoid divide by zero errors.
				when readBoth =>
					register8 <= "0000000010";
					pres_state <= reading;
			end case;
		end if;
	end process run;	
end architecture rtl; -- hall_sensor_accumulator