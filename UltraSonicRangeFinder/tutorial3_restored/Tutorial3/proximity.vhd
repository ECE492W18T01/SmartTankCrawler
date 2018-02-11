library ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_unsigned.all;
USE ieee.std_logic_signed.all;
use IEEE.numeric_std.all;
ENTITY proximity IS
  GENERIC(
      sys_clk         : INTEGER := 50_000_000; --system clock frequency in Hz
      cycle_time      : INTEGER := 49; --cycle time of the ultrasonic sensor in ms
	time_per_inch   : INTEGER := 147 -- pw time in us per one inch distance measurement
);
  PORT(
      	clk       : IN  STD_LOGIC := '0';                                    --system clock
      	reset_n   : IN  STD_LOGIC := '0';                                   --asynchronous reset                                 --latches in new duty cycle
	pw_signal : IN  STD_LOGIC := '0';
	avalon_slave_read_n : in  std_logic := '0'; -- avalon_slave.read_n
	proximity_inches_avalon_readdata: OUT STD_LOGIC_VECTOR(31 downto 0));
END proximity;

ARCHITECTURE logic OF proximity IS
  CONSTANT us_per_s : INTEGER := 1_000_000;
  CONSTANT ms_per_s : INTEGER := 1_000;
  CONSTANT clk_periods_per_sensor_cycle : INTEGER := (sys_clk/ms_per_s)*(cycle_time); 
  CONSTANT clk_periods_per_inch : INTEGER := (sys_clk/us_per_s)*time_per_inch;
  SIGNAL  clk_count        :  INTEGER RANGE 0 TO clk_periods_per_sensor_cycle; -- current clock count since the start of the sensor cycle
  SIGNAL  inch_count : INTEGER RANGE 0 TO clk_periods_per_sensor_cycle/clk_periods_per_inch; -- number of inches detected by proximity sensor

BEGIN
  PROCESS(clk, reset_n)
  BEGIN
    IF reset_n = '0' THEN                                                 --asynchronous reset
	clk_count <= 0;                                                --clear counter                                          --clear pwm inverse outputs
	inch_count <= 0;
   ELSIF rising_edge(clk) THEN                                      --rising system clock edge   
	IF clk_count = clk_periods_per_sensor_cycle - 1 THEN                       --end of sensor cycle reached
		clk_count <= 0;                                                        --reset counter
		proximity_inches_avalon_readdata <= std_logic_vector(to_unsigned(inch_count,32)); -- latch inch_count
		inch_count <= 0;                                                      -- reset inch_counter
        ELSE                                                                   --end of period not reached
		clk_count <= clk_count + 1;                                              --increment counter
        END IF;
	IF pw_signal = '1' THEN
		IF clk_count mod clk_periods_per_inch = 0 THEN
			inch_count <= inch_count + 1;
		END IF;
        END IF;
    END IF;
  END PROCESS;
  
END logic;