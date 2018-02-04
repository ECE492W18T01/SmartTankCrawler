-- hall_sensor_comparator.vhd

-- ECE 492 Wi 18 Team 1
-- Feb 1st, 2018
-- Keith Mills

-- Performs the arithmatic operations on the readings from the four hall sensors
-- Inputs:
-- start -> given by controller piece, to inform the module that the data has been received and to start operations
-- frontLeft, frontRight, rearLeft, rearRight -> Accumulated edge readings from four hall_sensor_accumulators.

-- Outputs:
-- frontRatio (rearRatio) -> frontLeft*256/frontRight (rearLeft*256/rearRight)
-- frontRearRatio -> (frontLeft+frontRight)*512/(rearLeft + rearRight)

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity hall_sensor_comparator is
	
	port (
		clk 				: in std_logic := '0';
		frontLeft			: in std_logic_vector(7 downto 0);
		frontRight			: in std_logic_vector(7 downto 0);
		rearLeft			: in std_logic_vector(7 downto 0);
		rearRight			: in std_logic_vector(7 downto 0);
		frontRatio			: out std_logic_vector(15 downto 0);
		rearRatio			: out std_logic_vector(15 downto 0);
		frontRearRatio			: out std_logic_vector(31 downto 0)
	);
	
end entity hall_sensor_comparator;

architecture rtl of hall_sensor_comparator is
	signal frontCalc: unsigned(15 downto 0) := "0000000000000000";
	signal rearCalc:  unsigned(15 downto 0) := "0000000000000000";
	signal totalCalc: unsigned(31 downto 0) := "00000000000000000000000000000000";
begin
	run:process(clk) is
	begin
		if rising_edge(clk) then
			-- Division syntax from https://stackoverflow.com/questions/21270074/division-in-vhdl
			frontCalc <= (to_unsigned(to_integer(unsigned(("00000000"&frontLeft)) * 256 / unsigned(frontRight)), 16));
			rearCalc <= (to_unsigned(to_integer(unsigned(("00000000"&rearLeft)) * 256 / unsigned(rearRight)), 16));
			totalCalc <= ((unsigned("00000000"&frontLeft) + unsigned("00000000"&frontRight))*512) /(unsigned(rearLeft) + unsigned(rearRight));
		end if;
		frontRatio <= std_logic_vector(frontCalc);
		rearRatio <= std_logic_vector(rearCalc);
		frontRearRatio <= std_logic_vector(totalCalc);
	end process run;

end architecture rtl; -- of hall_sensor_comparator