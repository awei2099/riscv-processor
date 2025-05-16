
`timescale 1ns / 1ns

module DividerUnsignedPipelined (
    input wire clk, rst,
    input  wire  [31:0] i_dividend,
    input  wire  [31:0] i_divisor,
    output logic [31:0] o_remainder,
    output logic [31:0] o_quotient
);

    logic [31:0] temp_dividend[33];
    logic [31:0] temp_quotient[33];
    logic [31:0] temp_remainder[33];

    logic [31:0] preg_dividend[9];
    logic [31:0] preg_divisor[9];
    logic [31:0] preg_quotient[9];
    logic [31:0] preg_remainder[9];    


	assign preg_dividend[0] = i_dividend;
	assign preg_divisor[0] = i_divisor;
	assign preg_quotient[0] = 32'b0;
	assign preg_remainder[0] = 32'b0;

	assign temp_dividend[0] = preg_dividend[0];
	assign temp_quotient[0] = preg_quotient[0];
	assign temp_remainder[0] = preg_remainder[0];


	genvar i;
    generate
		for (i = 0; i < 8; i = i + 1) begin 
			divu_1iter d1(
				.i_dividend(preg_dividend[i]),
				.i_divisor(preg_divisor[i]),
				.i_remainder(preg_remainder[i]),
				.i_quotient(preg_quotient[i]),
				.o_dividend(temp_dividend[4*i + 1]),
				.o_quotient(temp_quotient[4*i + 1]),
				.o_remainder(temp_remainder[4*i + 1])
			);
			
			divu_1iter d2(
				.i_dividend(temp_dividend[4*i + 1]),
				.i_divisor(preg_divisor[i]),
				.i_remainder(temp_remainder[4*i + 1]),
				.i_quotient(temp_quotient[4*i + 1]),
				.o_dividend(temp_dividend[4*i + 2]),
				.o_quotient(temp_quotient[4*i + 2]),
				.o_remainder(temp_remainder[4*i + 2])
			);
			
			divu_1iter d3(
				.i_dividend(temp_dividend[4*i + 2]),
				.i_divisor(preg_divisor[i]),
				.i_remainder(temp_remainder[4*i + 2]),
				.i_quotient(temp_quotient[4*i + 2]),
				.o_dividend(temp_dividend[4*i + 3]),
				.o_quotient(temp_quotient[4*i + 3]),
				.o_remainder(temp_remainder[4*i + 3])
			);
			
			logic [31:0] stage_dividend, stage_quotient, stage_remainder;
			divu_1iter d4(
				.i_dividend(temp_dividend[4*i + 3]),
				.i_divisor(preg_divisor[i]),
				.i_remainder(temp_remainder[4*i + 3]),
				.i_quotient(temp_quotient[4*i + 3]),
				.o_dividend(stage_dividend),
				.o_quotient(stage_quotient),
				.o_remainder(stage_remainder)
			);

			assign temp_dividend[4*i+4] = stage_dividend;
			assign temp_remainder[4*i+4] = stage_remainder;
			assign temp_quotient[4*i+4] = stage_quotient;

			always_ff @(posedge clk or posedge rst) begin
				if (rst) begin
					preg_dividend[i + 1] <= 32'b0;
                    preg_divisor[i + 1] <= 32'b0;
 					preg_quotient[i + 1] <= 32'b0;
					preg_remainder[i + 1] <= 32'b0;
				end
				else begin
					preg_dividend[i + 1] <= stage_dividend;
                    preg_divisor[i + 1] <= preg_divisor[i];
					preg_quotient[i + 1] <= stage_quotient;
					preg_remainder[i + 1] <= stage_remainder;					
				end
			end
		end
	endgenerate

	assign o_quotient = temp_quotient[32];
	assign o_remainder = temp_remainder[32];

endmodule


module divu_1iter (
    input  wire  [31:0] i_dividend,
    input  wire  [31:0] i_divisor,
    input  wire [31:0] i_remainder,
    input  wire [31:0] i_quotient,
    output logic [31:0] o_dividend,
    output logic [31:0] o_remainder,
    output logic [31:0] o_quotient
);
    wire [31:0] remainder;
    wire mux_ctrl;
    
    assign remainder = (i_remainder << 1) | ((i_dividend >> 31) & 1);
    assign mux_ctrl = remainder < i_divisor;

    assign o_dividend = i_dividend << 1;
    assign o_remainder = mux_ctrl ? (remainder) : (remainder - i_divisor);
    assign o_quotient = mux_ctrl ? (i_quotient << 1) : ((i_quotient << 1) | 1);
endmodule
