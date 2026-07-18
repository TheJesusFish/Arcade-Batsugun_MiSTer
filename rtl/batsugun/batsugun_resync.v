// Batsugun CRT sync offset shim, adapted from the local Raizing MiSTer core.
module batsugun_resync #(
    parameter H_DELAY = 0,
    parameter V_DELAY = 0
) (
    input            clk,
    input            pxl_cen,
    input            hs_in,
    input            vs_in,
    input            LVBL,
    input            LHBL,
    input      [4:0] hoffset,
    input      [3:0] voffset,
    output reg       hs_out,
    output reg       vs_out
);

reg [4:0] hs_pipe = 0;
reg [4:0] vs_pipe = 0;
reg [4:0] hs_sel;
reg [4:0] vs_sel;

always @* begin
    hs_sel = hoffset + H_DELAY[4:0];
    vs_sel = {1'b0, voffset} + V_DELAY[4:0];
end

always @(posedge clk) begin
    if (pxl_cen) begin
        hs_pipe <= {hs_pipe[3:0], hs_in};
        vs_pipe <= {vs_pipe[3:0], vs_in};
        hs_out <= hs_pipe[hs_sel[2:0]];
        vs_out <= vs_pipe[vs_sel[2:0]];
    end
end

endmodule
