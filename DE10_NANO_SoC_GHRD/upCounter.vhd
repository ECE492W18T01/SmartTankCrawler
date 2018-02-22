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

entity upCounter is
	port (
		avalon_slave_read_n   : in  std_logic                     := '0'; --  avalon_slave.read_n
		avalon_slave_readdata : out std_logic_vector(31 downto 0);        --              .readdata
		conduit_end_counter   : in  std_logic                     := '0';        --   conduit_end.export
		clk_counter           : in  std_logic                     := '0'; -- clock_counter.clk
		reset                 : in  std_logic                     := '0'  -- reset_counter.reset_n
	);
end entity upCounter;

architecture rtl of upCounter is
	signal counter: unsigned(31 downto 0) := "00000000000000000000000000011111";
begin

	adder:process(conduit_end_counter) is
	begin
		if rising_edge(conduit_end_counter) then
			counter <= counter + 1;
		--elsif rising_edge(reset) then
		--	counter <= "00000000000000000000000000000000";
		end if;
		avalon_slave_readdata <= std_logic_vector(counter);
	end process adder;
			
end architecture rtl; -- of new_component
