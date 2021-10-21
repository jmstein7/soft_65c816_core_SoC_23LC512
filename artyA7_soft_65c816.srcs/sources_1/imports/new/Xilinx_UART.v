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
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
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
