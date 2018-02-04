-- hall_sensor_comparator_tb.vhd

-- ECE 492 Wi18
-- Feb 3, 2018
-- Keith Mills

-- Testbench for hall_sensor_accumulator.vhd
-- Loads dummy values and then executes.

library ieee;
use ieee.std_logic_1164.all;

entity hall_sensor_comparator_tb is
end hall_sensor_comparator_tb;

architecture behave of hall_sensor_comparator_tb is
	signal clock:    std_logic := '0';
	constant CLOCK_period : time := 200ms;
	
	signal fL : std_logic_vector(7 downto 0) := "10001000";
	signal fR : std_logic_vector(7 downto 0) := "01001000";
	signal rL : std_logic_vector(7 downto 0) := "00101000";
	signal rR : std_logic_vector(7 downto 0) := "00011000";
	signal fRatio : std_logic_vector(15 downto 0) := "0000000000000000";
	signal rRatio : std_logic_vector(15 downto 0) := "0000000000000000";
	signal frRatio : std_logic_vector(31 downto 0) := "00000000000000000000000000000000";

	component hall_sensor_comparator is
		port (
			clk 				: in std_logic := '0';
			frontLeft			: in std_logic_vector(7 downto 0);
			frontRight			: in std_logic_vector(7 downto 0);
			rearLeft			: in std_logic_vector(7 downto 0);
			rearRight			: in std_logic_vector(7 downto 0);
			frontRatio			: out std_logic_vector(15 downto 0);
			rearRatio			: out std_logic_vector(15 downto 0);
			frontRearRatio			: out std_logic_vector(31 downto 0)
			--calcComplete			: out std_logic := '0'
		);
	end component hall_sensor_comparator;

begin
	OTR_INST : hall_sensor_comparator
		port map (
			clk => clock,
			frontLeft => fL,
			frontRight => fR,
			rearLeft => rL,
			rearRight => rR,
			frontRatio => fRatio,
			rearRatio => rRatio,
			frontRearRatio => frRatio
		);	

	CLOCK_process :process
	begin
		clock <= '1';
		wait for CLOCK_period/2;
		clock <= '0';
		wait for CLOCK_period/2;
	end process CLOCK_process;

	stim_proc: process
	begin
		fL <= "00001111";
		fR <= "00010000";
		rL <= "00001000";
		rR <= "01101000";
		wait for 150ms;
		fL <= "01100100";
		fR <= "01010000";
		rL <= "01101010";
		rR <= "01111010";
		wait for 200ms;
		fL <= "01010000";
		fR <= "01000100";
		rL <= "00100010";
		rR <= "01110010";
		wait for 200ms;
		fL <= "00001011";
		fR <= "00001110";
		rL <= "11001010";
		rR <= "11011000";
		wait for 200ms;
	end process stim_proc;
END;