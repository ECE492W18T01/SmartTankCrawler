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
		avalon_slave_read_n   : in  std_logic                     := '0'; --  avalon_slave.read_n
		avalon_slave_readdata : out std_logic_vector(31 downto 0);        --              .readdata
		conduit_end_counter   : in  std_logic                     := '0';        --   conduit_end.export
		clk_counter                   : in  std_logic                     := '0'; -- clock_counter.clk
		reset                 : in  std_logic                     := '0'  -- reset_counter.reset_n
	);
end entity upCounter;

architecture rtl of upCounter is
	signal counter: unsigned(31 downto 0) := "00000000000000000000000000000000";
	
	SIGNAL flipflops   : STD_LOGIC_VECTOR(1 DOWNTO 0); --input flip flops
	SIGNAL counter_set : STD_LOGIC;                    --sync reset to zero
	SIGNAL counter_out : STD_LOGIC_VECTOR(10 DOWNTO 0) := (OTHERS => '0'); --counter output
	SIGNAL dbswitch : STD_LOGIC; 
begin

	adder:process(dbswitch) is
	begin
		if rising_edge(dbswitch) then
			counter <= counter + 1;
		end if;
		avalon_slave_readdata  <= "00000000000000000000000000101000";
		--avalon_slave_readdata <= std_logic_vector(counter);
	end process adder;


process(clk_counter)
 --variable counter_size  :  INTEGER := 19;

BEGIN
    IF(clk_counter'EVENT and clk_counter = '1') THEN
      flipflops(0) <= conduit_end_counter;
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