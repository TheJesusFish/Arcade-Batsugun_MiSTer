// SPDX-License-Identifier: BSD-3-Clause

module batsugun_sound_mixer (
    input                clk,
    input                reset,
    input                sample,
    input  signed [15:0] ym_left,
    input  signed [15:0] ym_right,
    input  signed [13:0] oki,
    input                ym_enable,
    input                oki_enable,
    input                oki_ready,
    input                ss_restore_write,
    input         [35:0] ss_restore_data,
    output        [35:0] ss_state_data,
    output reg signed [15:0] mono
);

wire signed [19:0] ym_left_ext = ym_enable ?
                                  {{4{ym_left[15]}}, ym_left} : 20'sd0;
wire signed [19:0] ym_right_ext = ym_enable ?
                                   {{4{ym_right[15]}}, ym_right} : 20'sd0;

// MAME routes each YM channel and the OKI output at 0.5. JT6295's numeric
// output is eight times smaller than MAME's normalized OKI stream, so a
// coefficient of 16 before the final divide by two gives the same scale.
wire signed [19:0] oki_base = {{6{oki[13]}}, oki};
wire signed [19:0] oki_ext = oki_enable && oki_ready ?
                              (oki_base <<< 4) : 20'sd0;
wire signed [19:0] mono_sum = ym_left_ext + ym_right_ext + oki_ext;
wire signed [19:0] mono_scaled = mono_sum >>> 1;

// The chip streams update at 52.7 kHz while MiSTer consumes audio at 48 kHz.
// Average adjacent source samples to suppress zero-order-hold images before
// the framework samples the held result.
reg signed [19:0] previous_scaled = 20'sd0;
assign ss_state_data = {previous_scaled, mono};

wire signed [20:0] filter_sum =
    {{1{previous_scaled[19]}}, previous_scaled} +
    {{1{mono_scaled[19]}}, mono_scaled};
wire signed [20:0] mono_filtered = filter_sum >>> 1;

function signed [15:0] saturate16;
    input signed [20:0] value;
    begin
        if (value[20:15] == {6{value[15]}})
            saturate16 = value[15:0];
        else
            saturate16 = {value[20], {15{~value[20]}}};
    end
endfunction

always @(posedge clk) begin
    if (reset) begin
        previous_scaled <= 20'sd0;
        mono <= 16'sd0;
    end else if (ss_restore_write) begin
        previous_scaled <= ss_restore_data[35:16];
        mono <= ss_restore_data[15:0];
    end else if (sample) begin
        previous_scaled <= mono_scaled;
        mono <= saturate16(mono_filtered);
    end
end

endmodule
