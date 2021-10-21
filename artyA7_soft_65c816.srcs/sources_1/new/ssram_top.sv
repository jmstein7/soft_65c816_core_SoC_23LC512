`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 
// Engineer: Jonathan Stein
// 
// Create Date: 10/15/2021 02:54:38 PM
// Design Name: Quad SPI driver for 23LC512 Serial SRAM
// Module Name: ssram_top
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


module ssram_top(
    input logic spi_sck,
    input logic reset,
    input logic [15:0] address_in,
    input logic [7:0] data_in,
    input logic read,       //  HIGH for a single cycle 
    input logic write,      // to start operations (read or write)
    
    inout logic [3:0] sio,
    
    output logic read_busy,
    output logic write_busy,
    output logic sck,
    output logic csb,
    output logic [7:0] valid_data, 
    output logic data_valid,
    input logic byte_taken,
    output logic byte_ready
    );
    
    /* This is the address and address latch */ 
    logic [15:0] address; 
    
    /* These are the data values for reading and writing */
    logic [7:0] data_read;
    logic [7:0] data_to_write;  
    logic [3:0] dummy_byte;
    
    /* clock, timer, &c. */
    logic set_mode;
    logic [3:0] timer;
    
/* SQI Mode Set Sequence Reg/Wire */
   reg  si;
   wire [2:0] mode_sequence;
   
   logic ss = 1; 
   logic sio_0;
   logic sio_1;
   logic sio_2;
   logic sio_3;
   
   /* Control Logic */
   logic rwb = 0;
   logic start_read = 0;
   logic start_write = 0;
   
   assign sck = spi_sck; 
   
   /* Slave select and Tri-State Management */ 
   assign csb = (reset) ? 1'b1 : ss; 
   assign sio[0] = (~rwb) ? ((set_mode) ? si : sio_0) : 'bZ; 
   assign sio[1] = (~rwb) ? ((set_mode) ? 'bZ : sio_1) : 'bZ; 
   assign sio[2] = (~rwb) ? sio_2 : 'bZ;
   assign sio[3] = (~rwb) ? sio_3 : 'bZ;

   assign mode_sequence = timer[2:0]; 

   /* Parameters for entering sqi mode */
   parameter enter_sqi = 2'b00;
   parameter byte_read = 2'b01;
   parameter byte_write = 2'b10;
   parameter hold_sqi = 2'b11;

   reg [1:0] sqi_state = enter_sqi;

    /*----------------------
    /  Address Latch 
    /  with data write latch
    ----------------------*/
    
    always_latch begin
    
        if (spi_sck && (read || write)) begin
            address <= address_in;
            if (write)
                data_to_write <= data_in; 
        end
  
    //No else clause so a_latch's value
    //is not always defined, so it holds its value
    end

    /* Latch-in valid read data */
    always_latch begin
    
        if (data_valid) begin
            valid_data <= data_read;
            byte_ready <= 1; 
        end
        
        if (byte_taken)
            byte_ready <= 0;
            
        if (reset)
            byte_ready <= 0; 
  
    //No else clause so a_latch's value
    //is not always defined, so it holds its value
    end

   /*     Main Case Statement 
   /         #Control Case#
   /
   /
   */
   always @(posedge spi_sck)
      if (reset) begin
         timer <= 0; 
         set_mode <= 1; 
         ss <= 0; 
         read_busy <= 0; 
         write_busy <= 0;
         data_valid <= 0;
         sqi_state <= enter_sqi;
         
      end
      else
         case (sqi_state)
            enter_sqi : begin
               if (timer < 8) begin 
                  timer <= timer + 1;
                  sqi_state <= enter_sqi;
               end 
               else if (timer == 8) begin
                  timer <= 0; 
                  set_mode <= 0; 
                  ss <= 1;  
                  sqi_state <= hold_sqi;
               end

            end
            
            byte_read : begin
               if (~read && timer < 4'b1010) begin
                  timer <= timer + 1; 
                  sqi_state <= byte_read;
               end
               else begin 
                  data_valid <= 1;
                  ss <= 1; 
                  read_busy <= 0;
                  sqi_state <= hold_sqi;
               end
            end
    
            byte_write : begin
               if (~write && timer < 4'b1000) begin
                  timer <= timer + 1; 
                  sqi_state <= byte_write;
               end
               else begin
                  ss <= 1; 
                  write_busy <= 0; 
                  sqi_state <= hold_sqi;
               end
            end
            
            hold_sqi : begin
               if (read) begin
                  ss <= 0; 
                  timer <= 0; 
                  start_read <= 1;
                  read_busy <= 1;
                  sqi_state <= byte_read;
               end
               else if (write) begin
                  ss <= 0; 
                  timer <= 0; 
                  start_write <= 1;
                  write_busy <= 1; 
                  sqi_state <= byte_write;
               end
               else
                  if (data_valid)
                    data_valid <= 0;
                  sqi_state <= hold_sqi;
            end
         endcase


   /* These are the bits you write to put the device into Quad mode */
   always @(posedge spi_sck)
      if (set_mode)
         case (mode_sequence)
            4'b000: si <= 0;
            4'b001: si <= 0;
            4'b010: si <= 1;
            4'b011: si <= 1;
            4'b100: si <= 1;
            4'b101: si <= 0;
            4'b110: si <= 0;
            4'b111: si <= 0;
            default: si <= 0;
         endcase
    
    //----------- Begin Cut here for INSTANTIATION Template ---// INST_TAG
// INST_TAG_END ------ End INSTANTIATION Template ---------

/*
/       READ and WRITE BYTE STATE MACHINE
/
*/
        enum logic [4:0] {read_zero  = 5'b00000,
                     read_one  = 5'b00001,
                     read_two  = 5'b00010,
                     read_three  = 5'b00011,
                     read_four  = 5'b00100,
                     read_five  = 5'b00101,
                     read_six  = 5'b00110,
                     read_seven  = 5'b00111,
                     read_eight  = 5'b01000,
                     read_nine = 5'b01001,
                     read_write_hold = 5'b01010,
                     write_zero  = 5'b10000,
                     write_one  = 5'b10001,
                     write_two  = 5'b10010,
                     write_three  = 5'b10011,
                     write_four  = 5'b10100,
                     write_five  = 5'b10101,
                     write_six  = 5'b10110,
                     write_seven  = 5'b10111
                     } read_state;

   always @(posedge spi_sck)
      if (reset) begin
         read_state <= read_write_hold;
      end
      else
         case (read_state)
            read_zero : begin
                start_read <= 0; 
                sio_0 <= 0;
                sio_1 <= 0;
                sio_2 <= 0;
                sio_3 <= 0; 
                read_state <= read_one;
            end
            read_one : begin
                sio_0 <= 1;
                sio_1 <= 1;
                sio_2 <= 0;
                sio_3 <= 0; 
                read_state <= read_two;
            end
            read_two : begin
                sio_0 <= address[12];
                sio_1 <= address[13];
                sio_2 <= address[14];
                sio_3 <= address[15];
                read_state <= read_three;

            end
            read_three : begin
                sio_0 <= address[8];
                sio_1 <= address[9];
                sio_2 <= address[10];
                sio_3 <= address[11];
                read_state <= read_four;
            end
            read_four : begin
                sio_0 <= address[4];
                sio_1 <= address[5];
                sio_2 <= address[6];
                sio_3 <= address[7];
                read_state <= read_five;
            end
            read_five : begin
                sio_0 <= address[0];
                sio_1 <= address[1];
                sio_2 <= address[2];
                sio_3 <= address[3];
                rwb <= 1; 
                read_state <= read_six;
            end
            read_six : begin
                dummy_byte[0] <= sio[0];
                dummy_byte[1] <= sio[1];
                dummy_byte[2] <= sio[2];
                dummy_byte[3] <= sio[3]; 
                read_state <= read_seven;
            end
            read_seven : begin
                dummy_byte[0] <= sio[0];
                dummy_byte[1] <= sio[1];
                dummy_byte[2] <= sio[2];
                dummy_byte[3] <= sio[3]; 
                read_state <= read_eight;
            end
            read_eight : begin
                data_read[4] <= sio[0];
                data_read[5] <= sio[1];
                data_read[6] <= sio[2];
                data_read[7] <= sio[3]; 
                read_state <= read_nine;
            end
            read_nine : begin
                data_read[0] <= sio[0];
                data_read[1] <= sio[1];
                data_read[2] <= sio[2];
                data_read[3] <= sio[3]; 
                rwb <= 0;
                read_state <= read_write_hold;
            end
            /*                 HOLD                  */
            read_write_hold : begin
               if (~write && ~read)
                  read_state <= read_write_hold;
               else if (write)
                  read_state <= write_zero;
               else if (read)
                  read_state <= read_zero;
            end
            /*                END HOLD                   */
            write_zero : begin
                start_write <= 0; 
                sio_0 <= 0;
                sio_1 <= 0;
                sio_2 <= 0;
                sio_3 <= 0; 
                read_state <= write_one;
            end
            write_one : begin
                sio_0 <= 0;
                sio_1 <= 1;
                sio_2 <= 0;
                sio_3 <= 0; 
                read_state <= write_two;
            end
            write_two : begin
                sio_0 <= address[12];
                sio_1 <= address[13];
                sio_2 <= address[14];
                sio_3 <= address[15];
                read_state <= write_three;

            end
            write_three : begin
                sio_0 <= address[8];
                sio_1 <= address[9];
                sio_2 <= address[10];
                sio_3 <= address[11];
                read_state <= write_four;
            end
            write_four : begin
                sio_0 <= address[4];
                sio_1 <= address[5];
                sio_2 <= address[6];
                sio_3 <= address[7];
                read_state <= write_five;
            end
            write_five : begin
                sio_0 <= address[0];
                sio_1 <= address[1];
                sio_2 <= address[2];
                sio_3 <= address[3]; 
                read_state <= write_six;
            end
            write_six : begin
                sio_0 <= data_to_write[4];
                sio_1 <= data_to_write[5];
                sio_2 <= data_to_write[6];
                sio_3 <= data_to_write[7];  
                read_state <= write_seven;
            end
            write_seven : begin
                sio_0 <= data_to_write[0];
                sio_1 <= data_to_write[1];
                sio_2 <= data_to_write[2];
                sio_3 <= data_to_write[3];  
                read_state <= read_write_hold;
            end
   
         endcase
    
endmodule
