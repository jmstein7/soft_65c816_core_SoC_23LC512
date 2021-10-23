`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Jonathan Stein
// 
// Create Date: 10/16/2021 03:46:22 PM
// Design Name: 
// Module Name: serial_sram_driver
// Project Name: 
// Target Devices: Xilinx Arty A7 or other FPGAs or ASICs
// Tool Versions: 
// Description: Wrapper for Serial SRAM Driver
// 
// Dependencies: Serial SRAM Driver
// By Jonathan Stein
// Copyright (C) 2021
// Revision: 1.00
// Revision 0.01 - File Created
// Additional Comments:
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


module serial_sram_driver(
    input wire clk,  /*This is the input clock, equal to the sck output.  
                     //set to the spi clock rate */
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 reset RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_HIGH" *)
    input wire reset,    //Active HIGH
    
    input locked,
    input wire rwb,      //Set direction, active LOW
    input wire begin_operation,  //Once cycle HIGH to start ops
    
    inout wire [3:0] sio,
    input wire [15:0] address,
    input wire [7:0] data_in,
    output wire [7:0] data_out,
    
    output wire data_ready,
    output wire csb,
    output wire sck,
    
    input wire byte_taken,
    output wire byte_ready,
    output wire all_busy
    );
    
    wire write_busy;
    wire read_busy;
    reg read = 0;
    reg write = 0;

    assign all_busy = (read_busy || write_busy) ? 1'b1 : 1'b0;

    always @(posedge clk) begin
    
        if (read || write) begin
            read <= 0;
            write <= 0;
        end
        
        else if (~read && ~read_busy && begin_operation && rwb)
            read <= 1;
            
        else if (~write && ~write_busy && begin_operation && ~rwb)
            write <= 1;
        
    end

ssram_top serial_sram_instance(
    .spi_sck(clk),
    .reset(reset),
    
    .sio(sio),
    .address_in(address),
    .data_in(data_in),
    
    .read(read),       //  value of 1 for a single cycle 
    .write(write),      // to start operations (read or write)
    
    .read_busy(read_busy),
    .write_busy(write_busy),
    .sck(sck),
    .csb(csb),
    .valid_data(data_out), 
    .data_valid(data_ready),
    .byte_taken(byte_taken),
    .byte_ready(byte_ready)
    );
    
endmodule
