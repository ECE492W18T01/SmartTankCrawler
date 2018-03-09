-- hall_sensor_comparator.vhd

-- ECE 492 Wi 18 Team 1
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
		readFront	: in std_logic := '0'; -- Avalon read
		readRear 	: in std_logic := '0'; -- Avalon read
		readWhole	: in std_logic := '0'; -- Avalon read
		sysClk		: in std_logic := '0'; -- System clock 50MHz
		sampleImpulse	: in std_logic := '0'; -- Impulse from Sampling Clock
		fullClk 	: in std_logic := '0'; -- 2Hz from Samping Clock.
		halfClk		: in std_logic := '0'; -- 4Hz from sampling clock.
		reset		: in std_logic := '0'; -- Reset
		frontLeft	: in std_logic := '0'; -- Conduit
		frontRight	: in std_logic := '0'; -- Conduit
		rearLeft	: in std_logic := '0'; -- Conduit
		rearRight	: in std_logic := '0'; -- Conduit
		frontRatio	: out std_logic_vector(31 downto 0) := "00000000000000000000010000000000"; -- 1024
		rearRatio	: out std_logic_vector(31 downto 0) := "00000000000000000000010000000000"; -- 1024
		frontRearRatio	: out std_logic_vector(31 downto 0) := "00000000000000000000100000000000"; -- 2048  
		raiseInterrupt	: out std_logic := '0' -- IRQ Trigger
	);
	
end entity hall_sensor_comparator;

architecture rtl of hall_sensor_comparator is
	CONSTANT initialCalcs : unsigned := "00000000000000000000000000000001";
	CONSTANT initialConds : std_logic_vector(9 downto 0) := "0000000000";

	CONSTANT bitSize : Integer := 32;
	CONSTANT axleMult: Integer := 1024;

	CONSTANT overallMultWithPaddingZeros : unsigned(21 downto 0) := "0000000000100000000000";
	CONSTANT axlePaddingZeros : std_logic_vector(20 downto 0) := "000000000000000000000";

	signal frontCalc, rearCalc, totalCalc: unsigned(31 downto 0) := initialCalcs;
	
	signal fli, fri, rli, rri : std_logic_vector(9 downto 0) := initialConds; -- Difference between flc and flp
	signal fl, fr, rl, rr : std_logic_vector(9 downto 0) := initialConds; -- Floating values attached to accums
	signal flc, frc, rlc, rrc : std_logic_vector(9 downto 0) := initialConds; -- current value gotten at rising clock.
	signal flp, frp, rlp, rrp : std_logic_vector(9 downto 0) := initialConds; -- previous value. 

	component hall_sensor_accumulator is
		port (
			edge_input  : in  std_logic := '0';
			reset	    : in  std_logic := '0';
			sysClk	    : in  std_logic := '0';
			sendValue      : out std_logic_vector(9 downto 0)
		);
	end component hall_sensor_accumulator;
begin

	-- Wire the accumulators
	driverLeft : hall_sensor_accumulator
	port map (
		edge_input => frontLeft,
		reset      => reset,	
		sysClk	   => sysClk,
		sendValue     => fl
	);	

	driverRight : hall_sensor_accumulator
	port map (
		edge_input => frontRight,
		reset      => reset,
		sysClk	   => sysClk,
		sendValue     => fr
	);	

	backLeft : hall_sensor_accumulator
	port map (
		edge_input => rearLeft,
		reset      => reset,
		sysClk	   => sysClk,
		sendValue     => rl
	);	

	backRight : hall_sensor_accumulator
	port map (
		edge_input => rearRight,
		reset      => reset,
		sysClk	   => sysClk,
		sendValue     => rr
	);	

	getNewVals:process(fullClk) is
	begin
		if rising_edge(fullClk) then
			flc <= fl;
			frc <= fr;
			rlc <= rl;
			rrc <= rr;
		end if;
	end process getNewVals;
	
	fli <= std_logic_vector(unsigned(flc) - unsigned(flp) + 1);
	fri <= std_logic_vector(unsigned(frc) - unsigned(frp) + 1);
	rli <= std_logic_vector(unsigned(rlc) - unsigned(rlp) + 1);
	rri <= std_logic_vector(unsigned(rrc) - unsigned(rrp) + 1);
		
	calc:process(sampleImpulse) is
	begin	-- Division syntax from https://stackoverflow.com/questions/21270074/division-in-vhdl
		if rising_edge(sampleImpulse) then
			frontCalc <= (to_unsigned(to_integer(unsigned((axlePaddingZeros&fli)) * axleMult / unsigned(fri)), bitSize));
			rearCalc <= (to_unsigned(to_integer(unsigned((axlePaddingZeros&rli)) * axleMult / unsigned(rri)), bitSize));
			totalCalc <= ((unsigned(fli) + unsigned(fri)) * overallMultWithPaddingZeros) / (unsigned(rli) + unsigned(rri));
		end if;
	end process calc;
	
	
	
--	frontCalc <= (to_unsigned(to_integer(unsigned((axlePaddingZeros&fl)) * axleMult / unsigned(fr)), bitSize));
--	rearCalc <= (to_unsigned(to_integer(unsigned((axlePaddingZeros&rl)) * axleMult / unsigned(rr)), bitSize));
--	totalCalc <= ((unsigned(fl) + unsigned(fr)) * overallMultWithPaddingZeros) / (unsigned(rl) + unsigned(rr));

	-- Load data onto lines.
	postResultsAndRaiseInterrupt:process(halfClk) is
	begin
		if rising_edge(halfClk) and fullClk = '1' then
			frontRatio <= std_logic_vector(frontCalc);
			rearRatio <= std_logic_vector(rearCalc);
			frontRearRatio <= std_logic_vector(totalCalc);
			flp <= flc;
			frp <= frc;
			rlp <= rlc;
			rrp <= rrc;
		end if;
	end process postResultsAndRaiseInterrupt;
	
	-- New data arrives on the rising edge for fullClk, so by the rising edge for halfClk, there will be enough time for it to have been processed. 
	raiseInterrupt <= halfClk AND fullClk;
				
end architecture rtl; -- of hall_sensor_comparator