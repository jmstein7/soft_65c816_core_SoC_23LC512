`timescale 1ns / 1ps

module sram_ctrl5(clk, rw, wr_n, rd_n, ram_e, address_input, data_f2s, data_s2f, address_to_sram_output, we_to_sram_output, oe_to_sram_output, ce_to_sram_output, data_from_to_sram_input_output);

  input wire clk ;                                 //  Clock signal

  input wire rw;                                   //  With this signal, we select reading or writing operation
  input wire wr_n;
  input wire rd_n;
  input wire ram_e; 
  input wire [18:0] address_input;                 //  Address bus
  input wire [7:0] data_f2s;                       //  Data to be writteb in the SRAM

  output wire [7:0] data_s2f;                      //  It is the 8-bit registered data retrieved from the SRAM (the -s2f suffix stands for SRAM to FPGA)
  output wire [18:0] address_to_sram_output;        //  Address bus

  output wire we_to_sram_output;                    //  Write enable (active-low)
  output wire oe_to_sram_output;                    //  Output enable (active-low)
  output wire ce_to_sram_output;                    //  Chip enable (active-low). Disables or enables the chip.

  inout wire [7:0] data_from_to_sram_input_output; //  Data bus

  //	signal declaration
  wire start_operation;
  
  assign start_operation = (ram_e); 

  assign we_to_sram_output = wr_n;
  assign oe_to_sram_output = rd_n;
  assign ce_to_sram_output = (start_operation) ? 1'b0 : 1'b1;
  assign address_to_sram_output = address_input;
  
  assign data_from_to_sram_input_output = (start_operation && ~wr_n) ? data_f2s : 'bZ;
  assign data_s2f = (start_operation && ~rd_n) ? data_from_to_sram_input_output : 'bZ; 
  
endmodule  