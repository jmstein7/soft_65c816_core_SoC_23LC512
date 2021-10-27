`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/19/2021 07:37:36 PM
// Design Name: 
// Module Name: Xilinx_UART
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
//  MIT License
//
//  Copyright (c) 2021 Jonathan Stein (New York, USA)
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.
//////////////////////////////////////////////////////////////////////////////////


module Xilinx_UART(
  (* X_INTERFACE_INFO = "xilinx.com:interface:uart:1.0 <interface_name> RxD" *)
  input m_rxd, // Serial Input (required)
  (* X_INTERFACE_INFO = "xilinx.com:interface:uart:1.0 <interface_name> TxD" *)
  output m_txd, // Serial Output (required)
  (* X_INTERFACE_INFO = "xilinx.com:interface:uart:1.0 <interface_name> RTSn" *)
  output m_rtsn, // Request to Send (optional)
  (* X_INTERFACE_INFO = "xilinx.com:interface:uart:1.0 <interface_name> CTSn" *)
  input m_ctsn, // Clear to Send (optional)
//  additional ports here
  input acia_tx,
  output acia_rx
  
);

//  user logic here
assign m_txd = acia_tx;
assign acia_rx = m_rxd;

endmodule
