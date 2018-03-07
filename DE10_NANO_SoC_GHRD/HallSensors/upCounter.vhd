-- new_component.vhd

-- This file was auto-generated as a prototype implementation of a module
-- created in component editor.  It ties off all outputs to ground and
-- ignores all inputs.  It needs to be edited to make it do something
-- useful.
-- 
-- This file will not be automatically regenerated.  You should check it in
-- to your version control system if you want to keep it.

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use ieee.VITAL_Primitives.all;
USE ieee.std_logic_unsigned.all;

entity upCounter is
	port (
		readIn        : in  std_logic                     := '0'; --  avalon_slave.read_n
		sendValue     : out std_logic_vector(7 downto 0);        --              .readdata
		hallSensorIn  : in  std_logic                     := '0';        --   conduit_end.export
		sysClk        : in  std_logic                     := '0'; -- clock_counter.clk
		reset         : in  std_logic                     := '0'  -- reset_counter.reset_n
	);
end entity upCounter;

architecture rtl of upCounter is
	signal poscounter, negcounter: unsigned(7 downto 0) := "00000001";
	
	SIGNAL flipflops   : STD_LOGIC_VECTOR(1 DOWNTO 0); --input flip flops
	SIGNAL counter_set : STD_LOGIC;                    --sync reset to zero
	SIGNAL counter_out : STD_LOGIC_VECTOR(10 DOWNTO 0) := (OTHERS => '0'); --counter output
	SIGNAL dbswitch : STD_LOGIC; 
begin

	adder:process(dbswitch) is
	begin
		if rising_edge(dbswitch) then
			poscounter <= poscounter + 1;
		elsif falling_edge(dbswitch) then
			negcounter <= negcounter + 1;
		end if;
		sendValue <= std_logic_vector(poscounter + negcounter);
	end process adder;


process(sysClk)
 --variable counter_size  :  INTEGER := 19;

BEGIN
    IF(sysClk'EVENT and sysClk = '1') THEN
      flipflops(0) <= hallSensorIn;
      flipflops(1) <= flipflops(0);
      If(counter_set = '1') THEN                  --reset counter because input is changing
        counter_out <= (OTHERS => '0');
      ELSIF(counter_out(10) = '0') THEN --stable input time is not yet met
        counter_out <= counter_out + 1;
      ELSE                                        --stable input time is met
        dbswitch <= flipflops(1);
      END IF;    
    END IF;
	 
  END PROCESS;
  
  counter_set <= flipflops(0) xor flipflops(1); 
			
end architecture rtl; -- of new_component