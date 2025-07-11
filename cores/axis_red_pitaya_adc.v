`timescale 1 ns / 1 ps

module axis_red_pitaya_adc (
    // System signals
    input  wire        aclk,

    // ADC signals
    output wire        adc_csn,
    input  wire [15:0] adc_dat_a,
    input  wire [15:0] adc_dat_b,

    // Master AXI Stream interface
    output wire        m_axis_tvalid,
    output wire [31:0] m_axis_tdata
);

// Randomizer decode
wire [15:0] adc_dat_a_decoded;
wire [15:0] adc_dat_b_decoded;

genvar i;
generate
    for (i = 1; i < 16; i = i + 1) begin : decode_loop
        assign adc_dat_a_decoded[i] = adc_dat_a[i] ^ adc_dat_a[0];
        assign adc_dat_b_decoded[i] = adc_dat_b[i] ^ adc_dat_b[0];
    end
endgenerate

assign adc_dat_a_decoded[0] = adc_dat_a[0];
assign adc_dat_b_decoded[0] = adc_dat_b[0];

reg [15:0] dat_a_reg;
reg [15:0] dat_b_reg;

always @(posedge aclk) begin
    dat_a_reg <= adc_dat_a_decoded;
    dat_b_reg <= adc_dat_b_decoded;
end

assign adc_csn       = 1'b1;
assign m_axis_tvalid = 1'b1;

// I'm already using ADC in 2s complement
assign m_axis_tdata = {dat_b_reg, dat_a_reg};

endmodule