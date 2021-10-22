`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Jonathan Stein
// 
// Create Date: 10/20/2021 10:56:44 AM
// Design Name: 
// Module Name: top_level
// Project Name: 65c816 Complete SoC
// Target Devices: Xilinx Arty A7 or other FPGAs or ASICs
// Tool Versions: Vivado v2020.2
// Description: top level of the project
// 
// Dependencies: 
// 
// Revision:
// Revision 0.09 - File Created
// Additional Comments:
//
//   This work comprises a complete FPGA "System on a Chip" ("SoC")with a "soft" 
//   65c816 CPU Core at its center.
//
//   Copyright (C) 2021  Jonathan Stein, New York, USA
//
//   This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
//////////////////////////////////////////////////////////////////////////////////


module top_level(
    input sysclk,
    input acia_clk,
    input logic reset,
    
    //UART
    output wire uart_tx,
    input wire uart_rx,
    
    //SSRAM
    inout wire [3:0] sio,
    output wire csb,
    output wire sck,
    
    output logic led_a
    );
    
    //SSRAM
    logic ram_clk;
    logic ssram_enable;
    logic ssram_begin;
    logic ssram_valid;
    logic [7:0] ssram_in;
    logic [7:0] ssram_out; 
    logic [7:0] ssram_in_latch;
    logic [7:0] ssram_out_latch;
    logic byte_taken;
    logic byte_ready;
    logic ssram_busy;
    
    //ACIAs
    logic [7:0] acia_in;
    logic [7:0] acia_out; 
    logic acia_irq; 
    logic phi_enable; 
    logic [1:0] reg_select;
    logic acia_enable;
    //UART
    logic rts;
    logic tx;
    logic rx;
    logic cts;
    
    /* Main Control Signals */
    logic sclk; 
    logic phi2;
    logic we;
    logic rwb;
    logic [23:0] address; 
    logic [7:0] data_in;
    logic [7:0] data_out;
    logic vpa, vda, mlb, vpb, rdy_out,
    rom_enable, ram_enable, vpvda, db_enable, 
    data_rw, mwr_n, mrd_n; 
    
    /* Main Memories */
    logic [7:0] rom_out;
    logic [7:0] ram_out;
    logic [7:0] ram_in;
    
    logic resb,
    irqb,
    nmib,
    be,
    rdy,
    abort; 
    
    assign acia_e = ~acia_enable;
    /* Physical Switches and Lights */
    assign led_a = reset;
    assign resb = ~reset,
    irqb = 1,
    nmib = 1,
    be = 1,
    rdy = 1,
    abort = 1;
    
    assign vpvda = (vpa || vda) ? 1'b1 : 1'b0;  
    assign db_enable = (~phi2) ? 1'b0 : 1'b1; 
    assign mrw_enable = (phi2) ? 1'b1 : 1'b0;
    assign data_rw = (rwb) ? 1'b1 : 1'b0;
    //mrd and mrw signals
    assign mwr_n = (~rwb) ? 1'b0 : 1'b1; 
    assign mrd_n = (rwb) ? 1'b0 : 1'b1;
    
    /* Dynamic Memory Maps */
    assign rom_enable = ((address >= 24'h00C000 && address < 24'h010000) && vpvda) ? 1'b1 : 1'b0;
    assign ram_enable = ((address < 24'h008000) && vpvda) ? 1'b1 : 1'b0;
    
    /* Static Memory Maps */
    assign acia_enable = ((address >= 24'h008000 && address < 24'h008010) && vpvda) ? 1'b1 : 1'b0;
    assign ssram_enable = ((address >= 24'h010000 && address < 24'h020000) && vpvda) ? 1'b1 : 1'b0;
    
    /* In to Processor */
    assign data_in = (rwb) ? (rom_enable ? rom_out : (ram_enable ? ram_out : (acia_enable ? acia_out : (ssram_enable ? ssram_out : 'bZ)))) : 'bZ;
    
    /* Out from Processor */
    assign ram_in = (ram_enable && ~rwb) ? data_out : 'bZ;
    assign acia_in = (acia_enable && ~rwb) ? data_out : 'bZ;
    assign ssram_in = (ssram_enable && ~rwb) ? data_out : 'bZ;
    
    P65C816 cpu_one( 
        .CLK(phi2),			 //: in std_logic;
		.RST_N(resb),		      //: in std_logic;
		.CE(be),			        //: in std_logic;
		.RDY_IN(rdy),		     //: in std_logic;
        .NMI_N(nmib),		        //: in std_logic;  
		.IRQ_N(irqb),		      //: in std_logic; 
		.ABORT_N(abort),	     //: in std_logic;   -- just for WAI only
        .D_IN(data_in),		     //: in std_logic_vector(7 downto 0);
        .D_OUT(data_out),          //: out std_logic_vector(7 downto 0);
        .A_OUT(address),          //: out std_logic_vector(23 downto 0);
        .WE(rwb),  		     //: out std_logic; 
		.RDY_OUT(rdy_out), 	    //: out std_logic;
		.VPA(vpa), 		   //: out std_logic;
		.VDA(vda), 		       //: out std_logic;
		.MLB(mlb), 		   //: out std_logic;
		.VPB(vpb) 		   //: out std_logic
    );
    
  //SSRAM latch
  /////////////////
  always_latch begin
  
  if (ssram_enable && ~rwb && ~ssram_busy) begin
     ssram_in_latch = ssram_in;
  end
  else if (ssram_enable && rwb && ~byte_ready) begin
     ssram_out = 8'hEA;
  end
  else if (ssram_enable && rwb && byte_ready && ~byte_taken) begin
     ssram_out = ssram_out_latch;
     byte_taken = 1; 
  end
  else if (byte_taken)
    byte_taken = 0; 
  
  if (reset) begin
     ssram_in_latch = 8'h00;
     ssram_out = 8'h00;
     byte_taken = 1'b0;
  end
  
  //No else clause so a_latch's value
  //is not always defined, so it holds its value
end
   /////////////////

  clk_wiz_0 main_clocks
   (
    // Clock out ports
    .phi2(phi2),     // output phi2
    .sclk(sclk),     // output sclk
    .ram_clk(ram_clk),     // output ram_clk
    // Status and control signals
    .reset(reset), // input reset
    .locked(locked),       // output locked
   // Clock in ports
    .clk(sysclk));      // input clk

main_ROM your_instance_name (
  .clka(ram_clk),    // input wire clka
  .ena(rom_enable),      // input wire ena
  .addra(address[13:0]),  // input wire [13 : 0] addra
  .douta(rom_out)  // output wire [7 : 0] douta
);

Block_RAM_32k first_RAM_block (
  .clka(ram_clk),    // input wire clka
  .ena(ram_enable),      // input wire ena
  .wea(~rwb),      // input wire [0 : 0] wea
  .addra(address[14:0]),  // input wire [14 : 0] addra
  .dina(ram_in),    // input wire [7 : 0] dina
  .douta(ram_out)  // output wire [7 : 0] douta
);
  
  assign reg_select = address[1:0]; 
  
ACIA acia_a(
    .RESET(resb),
    .PHI2(phi2),
    .phi_enable(phi_enable),
    .CS(acia_e),
    .RWN(rwb),
    .RS(reg_select),
    .DATAIN(acia_in),
    .DATAOUT(acia_out),
    .XTLI(acia_clk),
    .RTSB(rts),
    .CTSB(cts),
    .DTRB(),
    .RXD(rx),
    .TXD(tx),
    .IRQn(acia_irq)
   );
   
   //UART Logic
Xilinx_UART UART_A(
  .m_rxd(uart_rx), // Serial Input (required)
  .m_txd(uart_tx), // Serial Output (required)
  .m_rtsn(cts), // Request to Send out(optional)
  .m_ctsn(rts), // Clear to Send in(optional)
//  additional ports here
  .acia_tx(tx),
  .acia_rx(rx)
);
   
   assign ssram_begin = (ssram_busy || (rwb && byte_ready) || ~ssram_enable) ? 1'b0 : 1'b1; 
   
serial_sram_driver ssram_one(
    .clk(sclk),  /*This is the input clock, equal to the sck output.  
                     //set to the spi clock rate */
    .reset(reset),    //Active HIGH
    
    .locked(),
    .rwb(rwb),      //Set direction, active LOW
    .begin_operation(ssram_begin),  //Once cycle HIGH to start ops
    
    .sio(sio),
    .address(address[15:0]),
    .data_in(ssram_in_latch),
    .data_out(ssram_out_latch),
    
    .data_ready(ssram_ready),
    .csb(csb),
    .sck(sck),
    
    .byte_taken(byte_taken),
    .byte_ready(byte_ready),
    .all_busy(ssram_busy)
    );
   
endmodule
