module Program_Counter(input [31:0]PC_in , output [31:0]PC_out , input clk , input reset );

initial
begin
PC_out = 0 ;
end

always @(posedge clock)
begin
if(reset)
        PC_out = 0 ;
else
   begin
        PC_out = PC_in ;
        assign PC_in = PC_in + 4 ;
   end
end

end module


module TEST () ;

reg [31:0]PC_out ;

wire clk ;
wire reset ;
wire  [31:0]PC_in ;

Program_Counter P1 ( PC_in , PC_out , clk , reset );

initial
begin
#10 PC_in = 32 ;
#10 PC_in = 45 ;

end
 
end module
