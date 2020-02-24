--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   22:47:38 09/11/2017
-- Design Name:   
-- Module Name:   E:/MBORD_0325_8MBx10_XFEL_CBPM_v2/M1/Project/fpga_top2_tb.vhd
-- Project Name:  FPGA_TOP
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: fpga_top2
-- 
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes: 
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation 
-- simulation model.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_arith.all;
 
use std.textio.all;
use work.data_conv_pkg_tb.all;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY fpga_top2_tb IS
generic (
		inp_file_1: string := "E:/MBORD_0325_8MBx10_XFEL_CBPM_v2/SIM/data/x_bin2.txt" ;   
		inp_file_2: string := "E:/MBORD_0325_8MBx10_XFEL_CBPM_v2/SIM/data/y_bin2.txt" ;   
		inp_file_3: string := "E:/MBORD_0325_8MBx10_XFEL_CBPM_v2/SIM/data/r_bin2.txt" ;  
		log_file1: string := "E:/MBORD_0325_8MBx10_XFEL_CBPM_v2/SIM/data/vx.txt" ;   
		log_file2: string := "E:/MBORD_0325_8MBx10_XFEL_CBPM_v2/SIM/data/px.txt" ;   
		log_file3: string := "E:/MBORD_0325_8MBx10_XFEL_CBPM_v2/SIM/data/vr.txt" ;  
		log_file4: string := "E:/MBORD_0325_8MBx10_XFEL_CBPM_v2/SIM/data/pr.txt"  
	 );
END fpga_top2_tb;
 
ARCHITECTURE behavior OF fpga_top2_tb IS 
	
	file output_file1 : text open write_mode is log_file1;
	file output_file2 : text open write_mode is log_file2;
	file output_file3 : text open write_mode is log_file3;
	file output_file4 : text open write_mode is log_file4;
	
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT fpga_top2
    PORT(
         adc_clk : IN  std_logic;
         adc1_data : IN  std_logic_vector(15 downto 0);
         adc2_data : IN  std_logic_vector(15 downto 0);
         adc3_data : IN  std_logic_vector(15 downto 0);
         adc4_data : IN  std_logic_vector(15 downto 0);
		 va    : out std_logic_vector(15 downto 0);
		 vb    : out std_logic_vector(15 downto 0);
		 vc    : out std_logic_vector(15 downto 0);
		 vd    : out std_logic_vector(15 downto 0);
		 v_valid : out std_logic
        );
    END COMPONENT;
    

   --Inputs
   signal adc_clk : std_logic := '0';
   signal adc1_data : std_logic_vector(15 downto 0) := (others => '0');
   signal adc2_data : std_logic_vector(15 downto 0) := (others => '0');
   signal adc3_data : std_logic_vector(15 downto 0) := (others => '0');
   signal adc4_data : std_logic_vector(15 downto 0) := (others => '0');
   
	signal va    	: std_logic_vector(15 downto 0);
	signal vb    	: std_logic_vector(15 downto 0);
	signal vc    	: std_logic_vector(15 downto 0);
	signal vd    	: std_logic_vector(15 downto 0);
	signal v_valid  : std_logic;

   -- Clock period definitions
   constant adc_clk_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: fpga_top2 PORT MAP (
          adc_clk => adc_clk,
          adc1_data => adc1_data,
          adc2_data => adc2_data,
          adc3_data => adc3_data,
          adc4_data => adc4_data,
		  va        => va     ,
		  vb        => vb     ,
		  vc        => vc     ,
		  vd        => vd     ,
		  v_valid   => v_valid
        );

   -- Clock process definitions
   adc_clk_process :process
   begin
		adc_clk <= '0';
		wait for adc_clk_period/2;
		adc_clk <= '1';
		wait for adc_clk_period/2;
   end process;
 

   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
      wait for 100 ns;	

      wait for adc_clk_period*10;

      -- insert stimulus here 

      wait;
   end process;
   
   file_input_1 : block
     file input_file_1 : text open read_mode is inp_file_1;
     begin
        process
           variable in_buf : line;
           variable vector_string : string(1 to 16);
        begin
           while not endfile(input_file_1) loop
              wait until rising_edge (adc_clk);        
                 in_buf := null;
                 readline(input_file_1, in_buf);
                 read(in_buf, vector_string);
                 adc1_data<= bin_string_to_std_logic_vector(vector_string, 16) after 0.01 ns;
           end loop;
      wait;
     end process;
   end block file_input_1; 
   
   file_input_2 : block
     file input_file_2 : text open read_mode is inp_file_2;
     begin
        process
           variable in_buf : line;
           variable vector_string : string(1 to 16);
        begin
           while not endfile(input_file_2) loop
              wait until rising_edge (adc_clk);        
                 in_buf := null;
                 readline(input_file_2, in_buf);
                 read(in_buf, vector_string);
                 adc2_data<= bin_string_to_std_logic_vector(vector_string, 16) after 0.01 ns;
           end loop;
      wait;
     end process;
   end block file_input_2; 
   
   file_input_3 : block
     file input_file_3 : text open read_mode is inp_file_3;
     begin
        process
           variable in_buf : line;
           variable vector_string : string(1 to 16);
        begin
           while not endfile(input_file_3) loop
              wait until rising_edge (adc_clk);        
                 in_buf := null;
                 readline(input_file_3, in_buf);
                 read(in_buf, vector_string);
                 adc4_data<= bin_string_to_std_logic_vector(vector_string, 16) after 0.01 ns;
           end loop;
      wait;
     end process;
   end block file_input_3; 
   
   --adc2_data <= adc1_data;
   adc3_data <= adc1_data;
   --adc4_data <= adc1_data;
   
   process (adc_clk)
   variable out_line : line;
   variable temp_data : std_logic_vector(15 downto 0);
   begin
   if rising_edge(adc_clk) then
      if (v_valid = '1') then
        write(out_line, std_logic_vector_to_bin_string(va, 16));
        writeline(output_file1, out_line);
      end if;
   end if;
   end process;
   
   process (adc_clk)
   variable out_line : line;
   variable temp_data : std_logic_vector(15 downto 0);
   begin
   if rising_edge(adc_clk) then
      if (v_valid = '1') then
        write(out_line, std_logic_vector_to_bin_string(vb, 16));
        writeline(output_file2, out_line);
      end if;
   end if;
   end process;
   
   process (adc_clk)
   variable out_line : line;
   variable temp_data : std_logic_vector(15 downto 0);
   begin
   if rising_edge(adc_clk) then
      if (v_valid = '1') then
        write(out_line, std_logic_vector_to_bin_string(vc, 16));
        writeline(output_file3, out_line);
      end if;
   end if;
   end process;
   
   process (adc_clk)
   variable out_line : line;
   variable temp_data : std_logic_vector(15 downto 0);
   begin
   if rising_edge(adc_clk) then
      if (v_valid = '1') then
        write(out_line, std_logic_vector_to_bin_string(vd, 16));
        writeline(output_file4, out_line);
      end if;
   end if;
   end process;

END;
