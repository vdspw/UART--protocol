// UART- top (contains instantiation of UART-tx and UART-rx).///////////
module uart_top #(parameter clk_freq = 1000000, 
                  parameter baud_rate = 9600)
  (
  	input clk,rst,
 	input rx,
  	input newd,
    input [7:0] dintx,
    
    output tx,
    output donetx,
    output [7:0] doutrx,
    output donerx
 
  );
  
  uarttx #(clk_freq,baud_rate) utx(clk,rst,newd,dintx,tx,donetx);
  
  uartrx #(clk_freq,baud_rate) rtx(clk,rst,rx,donerx,doutrx);

endmodule

//////////////////////////////////////////////////////////////////////////

////////// Transmitter////////////////////////////////////////////////////
module uarttx #(parameter clk_freq = 1000000, 
                 parameter baud_rate = 9600)
  (
    input clk,rst,
    input newd,
    input [7:0] tx_data,
    
    output reg tx,
    output reg donetx
 );
  
  localparam clkcount = (clk_freq/baud_rate);
  
  int counts = 0;
  int count = 0;
  
  reg uclk=0;
  
  enum bit[1:0] {idle = 2'b00, start = 2'b01, transfer = 2'b10, done = 2'b11	} state;
  //--UART clk gen--//
  
  always@(posedge clk)begin
    if(count < clkcount/2) begin
      count <= count + 1;
    end else begin
      count <= 0;
      uclk <= ~uclk;
    end
  end
  
  reg [7:0] din; // reg to store the tx_data.

  always@(posedge uclk) begin
    if(rst) begin
      state <= idle;
    end else begin
      case(state)
        idle:
          begin
            tx <= 1'b1; // start bit is HIGH
            count <= 0;
            donetx <= 1'b0;
            
            if(newd) begin
              state <= transfer;
              tx <= 1'b0;
              din <= tx_data ; // wrtie tx_data in the register.
            end else begin
              state <= idle;
            end
            
          end
		transfer:
          begin
            if(counts <=7 ) begin
              counts <= counts +1;
              tx <= din[counts]; // the index of the data 
			  state <= transfer;
            end else begin
              counts<=0;
              tx<= 1'b1;
              state <= idle;
              donetx <= 1'b1;
            end
            
          end
        
        default : state <= idle;
      endcase
    end
  end
  
endmodule

/////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////Reciever////////////////////////////////////////////////////////////////////

module uartrx #(parameter clk_freq = 1000000 ,
                parameter baud_rate = 9600 )
  (
    input clk,rst,
    input rx,
	output reg done,
    output reg [7:0] rxdata 
  );
  
  localparam clkcount = (clk_freq/baud_rate);
  
  int count = 0;
  int counts = 0;
  
  reg uclk=0;
  
  enum bit[1:0] {idle = 2'b00,start = 2'b01}state;
  
  ///UART clk/////
  always@(posedge clk) begin
    if (count < clkcount/2)begin
      count <= count +1;
      
    end
    else begin
      count <= 0;
      uclk <= ~uclk;
    end
  end
  
  always@(posedge uclk) begin
    if(rst)begin
      rxdata <= 8'b0;
      counts <= 0;
      done <= 1'b0;
    end else begin
      case(state)
        idle: begin
          rxdata <= 8'h00;
          counts <=0;
          done <= 1'b0;
          
          if(rx == 1'b0)
			state <= start;
          else
            state <= idle;
        end
        
        start: begin
          if(counts <=7)begin
            counts <= counts +1;
            rxdata <= {rx,rxdata[7:1]};
          end else begin
            counts <= 1'b0;
            done <= 1'b1;
            state <= idle;
          end
        end
        
        default : state<= idle;
      endcase
    end
  end
  
endmodule
///////////////////////////////////////////////////////////////////////////////////

interface uart_if;
  logic clk;
  logic rst;
  logic uclktx;
  logic uclkrx;
  logic rx;
  logic tx;
  logic [7:0] dintx;
  logic newd;
  logic [7:0] doutrx;
  logic donetx;
  logic donerx;
endinterface
