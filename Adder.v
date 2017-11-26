module MyAdder (in_1,in_2,out);
input [0:31] in_1,in_2;
output wire[31:0] out;
assign out=(in_1)+(in_2);
endmodule

module test_Adder;
reg [0:31] A, B;
wire [31:0] out;
MyAdder Myadd(A,B,out);
initial
begin
#10
A<=7;
B<=10;
$monitor ("in1=%d in2=%d out=%d",A, B,out);
#10
A<=19;
B<=10;
end
endmodule
