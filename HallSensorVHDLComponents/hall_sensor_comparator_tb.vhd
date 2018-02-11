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
	signal transmitClock: std_logic := '0';
	constant CLOCK_period : time := 200ms;

	signal fl, fr, rl, rr : std_logic := '0';
	constant fl_rate : time := 5ms;
	constant fr_rate : time := 2ms;
	constant rl_rate : time := 3ms;
	constant rr_rate : time := 9ms;

	signal fRatio, rRatio, oRatio : std_logic_vector(31 downto 0) := "00000000000000000000000000000000";

	signal intStatus : std_logic := '0';



	component hall_sensor_comparator is
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
	end component hall_sensor_comparator;

begin
	OTR_INST : hall_sensor_comparator
		port map (
			fullClk => clock,
			halfClk => transmitClock,
			reset => '0',
			frontLeft => fl,
			frontRight => fr,
			rearLeft => rl,
			rearRight => rr,
			frontRatio => fRatio,
			rearRatio => rRatio,
			frontRearRatio => oRatio,
			raiseInterrupt => intStatus
		);	

	CLOCK_process :process
	begin
		clock <= '1';
		wait for CLOCK_period/2;
		clock <= '0';
		wait for CLOCK_period/2;
	end process CLOCK_process;

	tCLOCK: process
	begin
		transmitClock <= '1';
		wait for CLOCK_period/4;
		transmitClock <= '0';
		wait for CLOCK_period/4;
	end process tCLOCK;
	
	driversLeft: process
	begin
		fl <= '0';
		wait for fl_rate/2;
		fl <= '1';
		wait for fl_rate/2;
	end process driversLeft;

	driversRight: process
	begin
		fr <= '0';
		wait for fr_rate/2;
		fr <= '1';
		wait for fr_rate/2;
	end process driversRight;

	backLeft: process
	begin
		rl <= '0';
		wait for rl_rate/2;
		rl <= '1';
		wait for rl_rate/2;
	end process backLeft;

	backRight: process
	begin
		rr <= '0';
		wait for rr_rate/2;
		rr <= '1';
		wait for rr_rate/2;
	end process backRight;
END;