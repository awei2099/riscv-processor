`timescale 1ns / 1ps

/**
 * @param a first 1-bit input
 * @param b second 1-bit input
 * @param g whether a and b generate a carry
 * @param p whether a and b would propagate an incoming carry
 */
module gp1(input wire a, b,
           output wire g, p);
   assign g = a & b;
   assign p = a | b;
endmodule

/**
 * Computes aggregate generate/propagate signals over a 4-bit window.
 * @param gin incoming generate signals
 * @param pin incoming propagate signals
 * @param cin the incoming carry
 * @param gout whether these 4 bits internally would generate a carry-out (independent of cin)
 * @param pout whether these 4 bits internally would propagate an incoming carry from cin
 * @param cout the carry outs for the low-order 3 bits
 */
module gp4(input wire [3:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [2:0] cout);
   assign gout = gin[3] | (pin[3] & gin[2]) | (pin[3] & pin[2] & gin[1]) | (pin[3] & pin[2] & pin[1] & gin[0]);
   assign pout = pin[3] & pin[2] & pin[1] & pin[0];

   assign cout[0] = gin[0] | (pin[0] & cin);
   assign cout[1] = gin[1] | (pin[1] & gin[0]) | (pin[1] & pin[0] & cin);
   assign cout[2] = gin[2] | (pin[2] & gin[1]) | (pin[2] & pin[1] & gin[0]) | (pin[2] & pin[1] & pin[0] & cin);
endmodule

/** Same as gp4 but for an 8-bit window instead */
module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);
   assign gout = gin[7] | (pin[7] & gin[6]) | (pin[7] & pin[6] & gin[5]) | 
                  (pin[7] & pin[6] & pin[5] & gin[4]) | 
                  (pin[7] & pin[6] & pin[5] & pin[4] & gin[3]) | 
                  (pin[7] & pin[6] & pin[5] & pin[4] & pin[3] & gin[2]) | 
                  (pin[7] & pin[6] & pin[5] & pin[4] & pin[3] & pin[2] & gin[1]) |
                  (pin[7] & pin[6] & pin[5] & pin[4] & pin[3] & pin[2] & pin[1] & gin[0]);

    assign pout = pin[7] & pin[6] & pin[5] & pin[4] & pin[3] & pin[2] & pin[1] & pin[0];

    assign cout[0] = gin[0] | (pin[0] & cin);
    assign cout[1] = gin[1] | (pin[1] & gin[0]) | (pin[1] & pin[0] & cin);
    assign cout[2] = gin[2] | (pin[2] & gin[1]) | (pin[2] & pin[1] & gin[0]) | (pin[2] & pin[1] & pin[0] & cin);
    assign cout[3] = gin[3] | (pin[3] & gin[2]) | (pin[3] & pin[2] & gin[1]) | 
                     (pin[3] & pin[2] & pin[1] & gin[0]) | (pin[3] & pin[2] & pin[1] & pin[0] & cin);
    assign cout[4] = gin[4] | (pin[4] & gin[3]) | (pin[4] & pin[3] & gin[2]) | 
                     (pin[4] & pin[3] & pin[2] & gin[1]) | (pin[4] & pin[3] & pin[2] & pin[1] & gin[0]) |
                     (pin[4] & pin[3] & pin[2] & pin[1] & pin[0] & cin);
    assign cout[5] = gin[5] | (pin[5] & gin[4]) | (pin[5] & pin[4] & gin[3]) | 
                     (pin[5] & pin[4] & pin[3] & gin[2]) | (pin[5] & pin[4] & pin[3] & pin[2] & gin[1]) | 
                     (pin[5] & pin[4] & pin[3] & pin[2] & pin[1] & gin[0]) |
                     (pin[5] & pin[4] & pin[3] & pin[2] & pin[1] & pin[0] & cin);
    assign cout[6] = gin[6] | (pin[6] & gin[5]) | (pin[6] & pin[5] & gin[4]) | 
                     (pin[6] & pin[5] & pin[4] & gin[3]) | (pin[6] & pin[5] & pin[4] & pin[3] & gin[2]) | 
                     (pin[6] & pin[5] & pin[4] & pin[3] & pin[2] & gin[1]) |
                     (pin[6] & pin[5] & pin[4] & pin[3] & pin[2] & pin[1] & gin[0]) |
                     (pin[6] & pin[5] & pin[4] & pin[3] & pin[2] & pin[1] & pin[0] & cin);
endmodule

`timescale 1ns / 1ps

module cla (
  input wire [31:0]  a, b,
  input wire         cin,
  output wire [31:0] sum
);

  wire [31:0] g, p;
  generate
    genvar i;
    for (i = 0; i < 32; i = i + 1) begin : gp1_loop
      gp1 u_gp1 (
        .a(a[i]),
        .b(b[i]),
        .g(g[i]),
        .p(p[i])
      );
    end
  endgenerate

  wire [3:0] G_block, P_block;
  wire [3:0] c_in_block;
  wire [6:0] cout_block0, cout_block1, cout_block2, cout_block3;

  gp8 block0 (
    .gin(g[7:0]),
    .pin(p[7:0]),
    .cin(c_in_block[0]),
    .gout(G_block[0]),
    .pout(P_block[0]),
    .cout(cout_block0)
  );

  gp8 block1 (
    .gin(g[15:8]),
    .pin(p[15:8]),
    .cin(c_in_block[1]),
    .gout(G_block[1]),
    .pout(P_block[1]),
    .cout(cout_block1)
  );

  gp8 block2 (
    .gin(g[23:16]),
    .pin(p[23:16]),
    .cin(c_in_block[2]),
    .gout(G_block[2]),
    .pout(P_block[2]),
    .cout(cout_block2)
  );

  gp8 block3 (
    .gin(g[31:24]),
    .pin(p[31:24]),
    .cin(c_in_block[3]),
    .gout(G_block[3]),
    .pout(P_block[3]),
    .cout(cout_block3)
  );

  assign c_in_block[0] = cin;
  assign c_in_block[1] = G_block[0] | (P_block[0] & cin);
  assign c_in_block[2] = G_block[1] | (P_block[1] & G_block[0]) | (P_block[1] & P_block[0] & cin);
  assign c_in_block[3] = G_block[2] | (P_block[2] & G_block[1]) | 
                         (P_block[2] & P_block[1] & G_block[0]) | 
                         (P_block[2] & P_block[1] & P_block[0] & cin);

  assign sum[0]   = a[0]  ^ b[0]  ^ c_in_block[0];
  assign sum[1]   = a[1]  ^ b[1]  ^ cout_block0[0];
  assign sum[2]   = a[2]  ^ b[2]  ^ cout_block0[1];
  assign sum[3]   = a[3]  ^ b[3]  ^ cout_block0[2];
  assign sum[4]   = a[4]  ^ b[4]  ^ cout_block0[3];
  assign sum[5]   = a[5]  ^ b[5]  ^ cout_block0[4];
  assign sum[6]   = a[6]  ^ b[6]  ^ cout_block0[5];
  assign sum[7]   = a[7]  ^ b[7]  ^ cout_block0[6];
  assign sum[8]   = a[8]  ^ b[8]  ^ c_in_block[1];
  assign sum[9]   = a[9]  ^ b[9]  ^ cout_block1[0];
  assign sum[10]  = a[10] ^ b[10] ^ cout_block1[1];
  assign sum[11]  = a[11] ^ b[11] ^ cout_block1[2];
  assign sum[12]  = a[12] ^ b[12] ^ cout_block1[3];
  assign sum[13]  = a[13] ^ b[13] ^ cout_block1[4];
  assign sum[14]  = a[14] ^ b[14] ^ cout_block1[5];
  assign sum[15]  = a[15] ^ b[15] ^ cout_block1[6];
  assign sum[16]  = a[16] ^ b[16] ^ c_in_block[2];
  assign sum[17]  = a[17] ^ b[17] ^ cout_block2[0];
  assign sum[18]  = a[18] ^ b[18] ^ cout_block2[1];
  assign sum[19]  = a[19] ^ b[19] ^ cout_block2[2];
  assign sum[20]  = a[20] ^ b[20] ^ cout_block2[3];
  assign sum[21]  = a[21] ^ b[21] ^ cout_block2[4];
  assign sum[22]  = a[22] ^ b[22] ^ cout_block2[5];
  assign sum[23]  = a[23] ^ b[23] ^ cout_block2[6];
  assign sum[24]  = a[24] ^ b[24] ^ c_in_block[3];
  assign sum[25]  = a[25] ^ b[25] ^ cout_block3[0];
  assign sum[26]  = a[26] ^ b[26] ^ cout_block3[1];
  assign sum[27]  = a[27] ^ b[27] ^ cout_block3[2];
  assign sum[28]  = a[28] ^ b[28] ^ cout_block3[3];
  assign sum[29]  = a[29] ^ b[29] ^ cout_block3[4];
  assign sum[30]  = a[30] ^ b[30] ^ cout_block3[5];
  assign sum[31]  = a[31] ^ b[31] ^ cout_block3[6];

endmodule


