-- hall_sensor_comparator.vhd

-- ECE 492 Wi 18 Team 1
-- Feb 1st, 2018
-- Keith Mills

-- Performs the arithmatic operations on the readings from the four hall sensors and posts the results to the memory locations, raising an interrupt.
-- Inputs:
-- fullClk -> Sampling clock to accumulators.
-- halfClk -> 2x speed of fullClk, controls when calculated data is posted to output lines and interrupts are raised.
-- frontLeft, frontRight, rearLeft, rearRight -> Accumulated edge readings from four hall_sensor_accumulators.

-- Outputs:
-- frontRatio (rearRatio) -> frontLeft*1024/frontRight (rearLeft*1024/rearRight)
-- frontRearRatio -> (frontLeft+frontRight)*2048/(rearLeft + rearRight)
-- raiseInterrupt -> ISR call.

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity hall_sensor_comparator is
	
	port (
		fullClk 			: in std_logic := '0';
		halfClk				: in std_logic := '0';
		reset				: in std_logic := '0';
		frontLeft			: in std_logic := '0';
		frontRight			: in std_logic := '0';
		rearLeft			: in std_logic := '0';
		rearRight			: in std_logic := '0';
		frontRatio			: out std_logic_vector(31 downto 0);
		rearRatio			: out std_logic_vector(31 downto 0);
		frontRearRatio			: out std_logic_vector(31 downto 0);
		raiseInterrupt			: out std_logic := '0'
	);
	
end entity hall_sensor_comparator;

architecture rtl of hall_sensor_comparator is
	signal frontCalc, rearCalc, totalCalc: unsigned(31 downto 0) := "00000000000000000000000000000000";
	signal fl, fr, rl, rr : std_logic_vector(9 downto 0) := "0000000000";
	signal secondDown : std_logic := '0';

	signal bitSize : Integer := 32;
	signal axleMult: Integer := 1024;

	signal overallMultWithPaddingZeros : unsigned(21 downto 0) := "0000000000100000000000";
	signal axlePaddingZeros : std_logic_vector(20 downto 0) := "000000000000000000000";

	component hall_sensor_accumulator is
		port (
			edge_input : in  std_logic := '0';
			reset	   : in  std_logic := '0';
			sampleClk  : in  std_logic := '0';
			output     : out std_logic_vector(9 downto 0)
		);
	end component hall_sensor_accumulator;
begin

	driverLeft : hall_sensor_accumulator
	port map (
		edge_input => frontLeft,
		reset      => reset,
		sampleClk  => fullClk,
		output     => fl
	);	

	driverRight : hall_sensor_accumulator
	port map (
		edge_input => frontRight,
		reset      => reset,
		sampleClk  => fullClk,
		output     => fr
	);	

	backLeft : hall_sensor_accumulator
	port map (
		edge_input => rearLeft,
		reset      => reset,
		sampleClk  => fullClk,
		output     => rl
	);	

	backRight : hall_sensor_accumulator
	port map (
		edge_input => rearRight,
		reset      => reset,
		sampleClk  => fullClk,
		output     => rr
	);	


	-- Division syntax from https://stackoverflow.com/questions/21270074/division-in-vhdl
	frontCalc <= (to_unsigned(to_integer(unsigned((axlePaddingZeros&fl)) * axleMult / unsigned(fr)), bitSize));
	rearCalc <= (to_unsigned(to_integer(unsigned((axlePaddingZeros&rl)) * axleMult / unsigned(rr)), bitSize));
	totalCalc <= ((unsigned(fl) + unsigned(fr)) * overallMultWithPaddingZeros) / (unsigned(rl) + unsigned(rr));


	postResultsAndRaiseInterrupt:process(halfClk) is
	begin
		if falling_edge(halfClk) and fullClk = '1' then
			frontRatio <= std_logic_vector(frontCalc);
			rearRatio <= std_logic_vector(rearCalc);
			frontRearRatio <= std_logic_vector(totalCalc);
			raiseInterrupt <= '1';
		elsif falling_edge(fullClk) and rising_edge(halfClk) then
			raiseInterrupt <= '0';
		end if;
	end process postResultsAndRaiseInterrupt;

end architecture rtl; -- of hall_sensor_comparator