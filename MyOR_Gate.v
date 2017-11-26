module MyOR_Gate(in_1,in_2,out);

input [0:31] in_1,in_2;
output wire[32:0] out;
assign out=(in_1) | (in_2);

endmodule

 module test_MyOR_Gate;
reg [0:31] in_1, in_2;
wire [31:0] out;
MyOR_Gate OR(in_1,in_2,out);
 initial
begin
in_1<=2256;
in_2<=10;
$monitor ("%b %b %b ", in_1,in_2,out);
end
endmodule
