-- hall_debouncer.vhd

-- Original file: switch_db.vhd
-- Nancy Minderman 
-- Created December 17, 2013
-- Makes much use of the Altera "Insert Template" feature
-- nancy.minderman@ualberta.ca
-- Modified January 13, 2014 to use the DE2_CONSTANTS package

-- hall_debouncer.vhd
-- ECE 492 Wi18
-- Team 1
-- Copy to supplied to Keith Mills (Ualberta ECE Student) by Wing Hoy 
-- (ualberta ECE lab tech) for project purposes.
-- Created February 23, 2018
-- https://eewiki.net/pages/viewpage.action?pageId=4980758#DebounceLogicCircuit(withVHDLexample)-Introduction


-- Signal debouncer for hall sensor.
-- Cleans up the input signal
-- Modified and "trimmed down" from the original version.

library ieee;
USE ieee.std_logic_unsigned.all; -- must use this for debounce 
-- Commonly imported packages:

-- STD_LOGIC and STD_LOGIC_VECTOR types, and relevant functions
use ieee.std_logic_1164.all;

-- SIGNED and UNSIGNED types, and relevant functions
use ieee.numeric_std.all;

-- Basic sequential functions and concurrent procedures
use ieee.VITAL_Primitives.all;

entity hall_debouncer is
	port (
		clk	  :	in std_logic;
		reset	  :	in std_logic;
		hallRough :	in std_logic;
		hallClean :	out std_logic := '0'
	);
end entity hall_debouncer;

architecture rtl of hall_debouncer is
	signal flipflops   : std_logic_vector(1 downto 0); --input flip flops
	signal counter_set : std_logic;                    --sync reset to zero
	signal counter_out : std_logic_vector(10 downto 0) := (others => '0'); --counter output
	signal dbswitch : std_logic;
begin

clean:process(clk)
begin
    if(clk'event and clk = '1') then
      flipflops(0) <= hallRough;
      flipflops(1) <= flipflops(0);
      If(counter_set = '1') then                  --reset counter because input is changing
        counter_out <= (others => '0');
      elsif(counter_out(10) = '0') then --stable input time is not yet met
        counter_out <= counter_out + 1;
      else                                        --stable input time is met
        hallClean <= flipflops(1);
      end if;    
    end if;
	 
  end process clean;
  
  counter_set <= flipflops(0) xor flipflops(1); 
			
end architecture rtl; -- of hall_debouncer