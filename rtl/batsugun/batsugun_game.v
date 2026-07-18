// Early Batsugun bring-up wrapper.
// This is intentionally a video/ROM-load baseline while the TP-030 board logic
// is built from the local MAME references and the V25 VHDL core.
module batsugun_game #(
    parameter AW = 22,
    parameter [1:0] GP_LIVE_GFX_DECODE_MODE = 2'd3,
    parameter [1:0] GP_LIVE_GFX_MAP_MODE = 2'd1,
    parameter [1:0] GP_LIVE_TILE_WORD_MODE = 2'd0
) (
    input              rst,
    input              clk,
    input              clk96,
    input              rst96,
    input              clk48,
    input              rst48,
    input              clk24,
    input              rst24,

    output reg         pxl2_cen,
    output reg         pxl_cen,
    output reg [7:0]   red,
    output reg [7:0]   green,
    output reg [7:0]   blue,
    output reg         LHBL,
    output reg         LVBL,
    output reg         HS,
    output reg         VS,

    input      [3:0]   cab_1p,
    input      [3:0]   coin,
    input      [`JTFRAME_BUTTONS+3:0] joystick1,
    input      [`JTFRAME_BUTTONS+3:0] joystick2,
    input      [`JTFRAME_BUTTONS+3:0] joystick3,
    input      [`JTFRAME_BUTTONS+3:0] joystick4,
    input      [15:0]  joyana_l1,
    input      [15:0]  joyana_l2,
    input      [15:0]  joyana_l3,
    input      [15:0]  joyana_l4,
    input      [15:0]  joyana_r1,
    input      [15:0]  joyana_r2,
    input      [15:0]  joyana_r3,
    input      [15:0]  joyana_r4,
    input      [1:0]   dial_x,
    input      [1:0]   dial_y,

    input      [26:0]  ioctl_addr,
    input      [7:0]   ioctl_dout,
    input              ioctl_cart,
    input              ioctl_wr,
    input              ioctl_ram,
    output     [7:0]   ioctl_din,
    input              ioctl_rom,
    output             dwnld_busy,
    input      [15:0]  data_read,

    output     [AW-1:0] ba0_addr,
    output     [AW-1:0] ba1_addr,
    output     [AW-1:0] ba2_addr,
    output     [AW-1:0] ba3_addr,
    output     [3:0]   ba_rd,
    output     [3:0]   ba_wr,
    input      [3:0]   ba_dst,
    input      [3:0]   ba_dok,
    input      [3:0]   ba_rdy,
    input      [3:0]   ba_ack,
    output     [15:0]  ba0_din,
    output     [1:0]   ba0_dsn,
    output     [15:0]  ba1_din,
    output     [1:0]   ba1_dsn,
    output     [15:0]  ba2_din,
    output     [1:0]   ba2_dsn,
    output     [15:0]  ba3_din,
    output     [1:0]   ba3_dsn,

    output     [1:0]   prog_ba,
    input              prog_rdy,
    input              prog_ack,
    input              prog_dok,
    input              prog_dst,
    output     [15:0]  prog_data,
    output     [AW-1:0] prog_addr,
    output             prog_rd,
    output             prog_we,
    output     [1:0]   prog_mask,

    input      [31:0]  status,
    input              dip_pause,
    inout              dip_flip,
    input              dip_test,
    input      [1:0]   dip_fxlevel,
    input              service,
    input              tilt,
    input      [31:0]  dipsw,

    output signed [15:0] snd_left,
    output signed [15:0] snd_right,
    output             sample,
    input      [5:0]   snd_en,
    input      [7:0]   snd_vol,
    output     [5:0]   snd_vu,
    output             snd_peak,

    input      [3:0]   gfx_en,
    output     [7:0]   debug_bus,
    output     [7:0]   debug_view,

    input              ss_do_save,
    input              ss_do_restore,
    input              ss_busy,
    input              ss_format_valid,
    output reg         ss_write_start,
    output reg         ss_read_start,
    output             ss_active,
    output     [3:0]   ss_state_out,
    input      [63:0]  ss_data,
    input      [31:0]  ss_addr,
    input      [7:0]   ss_select,
    input              ss_write,
    input              ss_read,
    input              ss_query,
    output     [63:0]  ss_data_out,
    output             ss_ack
);

localparam H_TOTAL = 10'd432;
localparam H_START = 10'd0;
localparam H_END   = 10'd320;
localparam V_TOTAL = 9'd262;
localparam V_START = 9'd0;
localparam V_END   = 9'd240;
localparam H_PREFETCH_START = H_TOTAL - 10'd32;
localparam [8:0] GP_HBLANK_PREFETCH_X_ADD = 9'd80; // hcnt 400..431 -> x 480..511
localparam OBJ_CACHE_SCAN_V = 9'd258;
localparam OBJ_CACHE_SCAN_H = 10'd0;
localparam OBJ_SCROLL_BYPASS_TEST = 1'b0;
localparam OBJ_VISIBLE_CACHE_TEST = 1'b1;
localparam OBJ_LINE_SECONDARY_FETCH = 1'b1;
localparam ENABLE_OBJ_LINEBUFFER_COMPOSITE = 1'b1;
localparam OBJ_PRIORITY_EVEN_MASK_TEST = 1'b1;
localparam [8:0] OBJ_LB_URGENT_LINES = 9'd16;
localparam COMP_MISS_HOLD_LAST_TEST = 1'b0;
localparam [6:0] PRESSURE_HOLD_FRAMES = 7'd119;
localparam COMP_BANKED_SCROLL_DIAG = 1'b0;
localparam COMP_BANK_OBJECTS_DIAG = 1'b0;

localparam [3:0] SS_IDLE               = 4'd0;
localparam [3:0] SS_SAVE_WAIT_SAFE     = 4'd1;
localparam [3:0] SS_SAVE_WAIT_IRQ      = 4'd2;
localparam [3:0] SS_SAVE_WAIT_SSP      = 4'd3;
localparam [3:0] SS_SAVE_WAIT_STREAM   = 4'd4;
localparam [3:0] SS_SAVE_WAIT_EXIT     = 4'd5;
localparam [3:0] SS_RESTORE_WAIT_SAFE  = 4'd6;
localparam [3:0] SS_RESTORE_WAIT_STREAM= 4'd7;
localparam [3:0] SS_RESTORE_HOLD_RESET = 4'd8;
localparam [3:0] SS_RESTORE_WAIT_RESET = 4'd9;
localparam [3:0] SS_SAVE_WAIT_HOLD     = 4'd10;
localparam [3:0] SS_RESTORE_WAIT_HOLD  = 4'd11;
localparam [3:0] SS_RESTORE_WAIT_VIDEO = 4'd12;
localparam [3:0] SS_SAVE_WAIT_VIDEO    = 4'd13;

localparam [7:0] SSIDX_GLOBAL = 8'd1;

reg [3:0] ss_state = SS_IDLE;
reg [7:0] ss_reset_counter = 8'd0;
reg [31:0] ss_saved_ssp = 32'd0;
reg [31:0] ss_restore_ssp = 32'd0;
reg [63:0] ss_restore_rom_signature = 64'd0;
reg        ss_restore_irq4 = 1'b0;
reg        ss_restore_sound_released = 1'b0;
reg [7:0]  ss_restore_bgm_command = 8'h00;
reg [7:0]  ss_restore_bgm_argument = 8'h00;
reg        ss_restore_bgm_valid = 1'b0;
reg        ss_restore_commit = 1'b0;
reg        ss_video_frame_seen = 1'b0;
reg [15:0] ss_reset_vector [0:3];

reg [7:0] sound_bgm_pending_argument = 8'h00;
reg [7:0] sound_bgm_command = 8'h00;
reg [7:0] sound_bgm_argument = 8'h00;
reg       sound_bgm_valid = 1'b0;

wire ss_irq = ss_state == SS_SAVE_WAIT_IRQ;
wire ss_override = (ss_state == SS_SAVE_WAIT_SSP) ||
                   (ss_state == SS_SAVE_WAIT_EXIT) ||
                   (ss_state == SS_RESTORE_HOLD_RESET) ||
                   (ss_state == SS_RESTORE_WAIT_RESET);
wire ss_reset = ss_state == SS_RESTORE_HOLD_RESET;
wire ss_cpu_run = (ss_state != SS_SAVE_WAIT_STREAM) &&
                  (ss_state != SS_RESTORE_WAIT_STREAM) &&
                  (ss_state != SS_SAVE_WAIT_HOLD) &&
                  (ss_state != SS_RESTORE_WAIT_HOLD) &&
                  (ss_state != SS_SAVE_WAIT_VIDEO) &&
                  (ss_state != SS_RESTORE_WAIT_VIDEO);
wire ss_v25_run = (ss_state == SS_IDLE) ||
                  (ss_state == SS_SAVE_WAIT_SAFE) ||
                  (ss_state == SS_RESTORE_WAIT_SAFE);
wire ss_device_hold = !ss_v25_run;
wire ss_video_reset =
    (ss_state == SS_SAVE_WAIT_IRQ) ||
    (ss_state == SS_SAVE_WAIT_SSP) ||
    (ss_state == SS_SAVE_WAIT_HOLD) ||
    (ss_state == SS_SAVE_WAIT_STREAM) ||
    (ss_state == SS_RESTORE_WAIT_HOLD) ||
    (ss_state == SS_RESTORE_WAIT_STREAM);

assign ss_active = ss_state != SS_IDLE;
assign ss_state_out = ss_state;

function [15:0] ss_irq_handler_word;
    input [3:0] index;
    begin
        case (index)
            4'h0: ss_irq_handler_word = 16'h48e7;
            4'h1: ss_irq_handler_word = 16'hfffe;
            4'h2: ss_irq_handler_word = 16'h4e6e;
            4'h3: ss_irq_handler_word = 16'h2f0e;
            4'h4: ss_irq_handler_word = 16'h4df9;
            4'h5: ss_irq_handler_word = 16'h00ff;
            4'h6: ss_irq_handler_word = 16'h0000;
            4'h7: ss_irq_handler_word = 16'h2c8f;
            4'h8: ss_irq_handler_word = 16'h2c5f;
            4'h9: ss_irq_handler_word = 16'h4e66;
            4'ha: ss_irq_handler_word = 16'h4cdf;
            4'hb: ss_irq_handler_word = 16'h7fff;
            4'hc: ss_irq_handler_word = 16'h4e73;
            default: ss_irq_handler_word = 16'h0000;
        endcase
    end
endfunction

function [3:0] hex_glyph;
    input [3:0] digit;
    input [2:0] row;
    begin
        case (digit)
            4'h0: case (row)
                3'd0: hex_glyph = 4'b1111;
                3'd1: hex_glyph = 4'b1001;
                3'd2: hex_glyph = 4'b1001;
                3'd3: hex_glyph = 4'b1001;
                3'd4: hex_glyph = 4'b1001;
                3'd5: hex_glyph = 4'b1001;
                default: hex_glyph = 4'b1111;
            endcase
            4'h1: case (row)
                3'd0: hex_glyph = 4'b0010;
                3'd1: hex_glyph = 4'b0110;
                3'd2: hex_glyph = 4'b0010;
                3'd3: hex_glyph = 4'b0010;
                3'd4: hex_glyph = 4'b0010;
                3'd5: hex_glyph = 4'b0010;
                default: hex_glyph = 4'b0111;
            endcase
            4'h2: case (row)
                3'd0: hex_glyph = 4'b1111;
                3'd1: hex_glyph = 4'b0001;
                3'd2: hex_glyph = 4'b0001;
                3'd3: hex_glyph = 4'b1111;
                3'd4: hex_glyph = 4'b1000;
                3'd5: hex_glyph = 4'b1000;
                default: hex_glyph = 4'b1111;
            endcase
            4'h3: case (row)
                3'd0: hex_glyph = 4'b1111;
                3'd1: hex_glyph = 4'b0001;
                3'd2: hex_glyph = 4'b0001;
                3'd3: hex_glyph = 4'b1111;
                3'd4: hex_glyph = 4'b0001;
                3'd5: hex_glyph = 4'b0001;
                default: hex_glyph = 4'b1111;
            endcase
            4'h4: case (row)
                3'd0: hex_glyph = 4'b1001;
                3'd1: hex_glyph = 4'b1001;
                3'd2: hex_glyph = 4'b1001;
                3'd3: hex_glyph = 4'b1111;
                3'd4: hex_glyph = 4'b0001;
                3'd5: hex_glyph = 4'b0001;
                default: hex_glyph = 4'b0001;
            endcase
            4'h5: case (row)
                3'd0: hex_glyph = 4'b1111;
                3'd1: hex_glyph = 4'b1000;
                3'd2: hex_glyph = 4'b1000;
                3'd3: hex_glyph = 4'b1111;
                3'd4: hex_glyph = 4'b0001;
                3'd5: hex_glyph = 4'b0001;
                default: hex_glyph = 4'b1111;
            endcase
            4'h6: case (row)
                3'd0: hex_glyph = 4'b1111;
                3'd1: hex_glyph = 4'b1000;
                3'd2: hex_glyph = 4'b1000;
                3'd3: hex_glyph = 4'b1111;
                3'd4: hex_glyph = 4'b1001;
                3'd5: hex_glyph = 4'b1001;
                default: hex_glyph = 4'b1111;
            endcase
            4'h7: case (row)
                3'd0: hex_glyph = 4'b1111;
                3'd1: hex_glyph = 4'b0001;
                3'd2: hex_glyph = 4'b0010;
                3'd3: hex_glyph = 4'b0010;
                3'd4: hex_glyph = 4'b0100;
                3'd5: hex_glyph = 4'b0100;
                default: hex_glyph = 4'b0100;
            endcase
            4'h8: case (row)
                3'd0: hex_glyph = 4'b1111;
                3'd1: hex_glyph = 4'b1001;
                3'd2: hex_glyph = 4'b1001;
                3'd3: hex_glyph = 4'b1111;
                3'd4: hex_glyph = 4'b1001;
                3'd5: hex_glyph = 4'b1001;
                default: hex_glyph = 4'b1111;
            endcase
            4'h9: case (row)
                3'd0: hex_glyph = 4'b1111;
                3'd1: hex_glyph = 4'b1001;
                3'd2: hex_glyph = 4'b1001;
                3'd3: hex_glyph = 4'b1111;
                3'd4: hex_glyph = 4'b0001;
                3'd5: hex_glyph = 4'b0001;
                default: hex_glyph = 4'b1111;
            endcase
            4'ha: case (row)
                3'd0: hex_glyph = 4'b1111;
                3'd1: hex_glyph = 4'b1001;
                3'd2: hex_glyph = 4'b1001;
                3'd3: hex_glyph = 4'b1111;
                3'd4: hex_glyph = 4'b1001;
                3'd5: hex_glyph = 4'b1001;
                default: hex_glyph = 4'b1001;
            endcase
            4'hb: case (row)
                3'd0: hex_glyph = 4'b1110;
                3'd1: hex_glyph = 4'b1001;
                3'd2: hex_glyph = 4'b1001;
                3'd3: hex_glyph = 4'b1110;
                3'd4: hex_glyph = 4'b1001;
                3'd5: hex_glyph = 4'b1001;
                default: hex_glyph = 4'b1110;
            endcase
            4'hc: case (row)
                3'd0: hex_glyph = 4'b1111;
                3'd1: hex_glyph = 4'b1000;
                3'd2: hex_glyph = 4'b1000;
                3'd3: hex_glyph = 4'b1000;
                3'd4: hex_glyph = 4'b1000;
                3'd5: hex_glyph = 4'b1000;
                default: hex_glyph = 4'b1111;
            endcase
            4'hd: case (row)
                3'd0: hex_glyph = 4'b1110;
                3'd1: hex_glyph = 4'b1001;
                3'd2: hex_glyph = 4'b1001;
                3'd3: hex_glyph = 4'b1001;
                3'd4: hex_glyph = 4'b1001;
                3'd5: hex_glyph = 4'b1001;
                default: hex_glyph = 4'b1110;
            endcase
            4'he: case (row)
                3'd0: hex_glyph = 4'b1111;
                3'd1: hex_glyph = 4'b1000;
                3'd2: hex_glyph = 4'b1000;
                3'd3: hex_glyph = 4'b1110;
                3'd4: hex_glyph = 4'b1000;
                3'd5: hex_glyph = 4'b1000;
                default: hex_glyph = 4'b1111;
            endcase
            default: case (row)
                3'd0: hex_glyph = 4'b1111;
                3'd1: hex_glyph = 4'b1000;
                3'd2: hex_glyph = 4'b1000;
                3'd3: hex_glyph = 4'b1110;
                3'd4: hex_glyph = 4'b1000;
                3'd5: hex_glyph = 4'b1000;
                default: hex_glyph = 4'b1000;
            endcase
        endcase
end
endfunction

function [7:0] sat8_inc;
    input [7:0] value;
    begin
        sat8_inc = (value == 8'hff) ? 8'hff : (value + 8'h01);
    end
endfunction

function [7:0] max8;
    input [7:0] a;
    input [7:0] b;
    begin
        max8 = (a > b) ? a : b;
    end
endfunction

function [12:0] gp_layer_base;
    input [1:0] layer;
    begin
        gp_layer_base = (layer == 2'd0) ? 13'h0000 :
                        (layer == 2'd1) ? 13'h0800 :
                                           13'h1000;
    end
endfunction

function [12:0] gp_tile_pair_addr_calc;
    input [1:0] layer;
    input [4:0] row;
    input [4:0] col;
    begin
        gp_tile_pair_addr_calc = gp_layer_base(layer) + {2'b00, row, col, 1'b0};
    end
endfunction

function [4:0] gp_word_in_tile_calc;
    input [3:0] py;
    input       right_half;
    reg [4:0] row_base;
    begin
        row_base = py[3] ? (5'd16 + {2'b00, py[2:0]}) :
                           {2'b00, py[2:0]};
        gp_word_in_tile_calc = row_base + (right_half ? 5'd8 : 5'd0);
    end
endfunction

function [15:0] gp_attr_eff_calc;
    input [15:0] attr_word;
    input [15:0] code_word;
    input [1:0]  word_mode;
    reg [15:0] attr_pair;
    begin
        attr_pair = word_mode[1] ? code_word : attr_word;
        gp_attr_eff_calc = word_mode[0] ? {attr_pair[7:0], attr_pair[15:8]} :
                                           attr_pair;
    end
endfunction

function [15:0] gp_code_eff_calc;
    input [15:0] attr_word;
    input [15:0] code_word;
    input [1:0]  word_mode;
    reg [15:0] code_pair;
    begin
        code_pair = word_mode[1] ? attr_word : code_word;
        gp_code_eff_calc = word_mode[0] ? {code_pair[7:0], code_pair[15:8]} :
                                           code_pair;
    end
endfunction

function [AW-1:0] gp_gfx_word_offset_calc;
    input        gp_sel;
    input [15:0] tile_code;
    input [4:0]  word_in_tile;
    reg [20:0] gp0_offset;
    reg [19:0] gp1_offset;
    begin
        gp0_offset = {1'b0, tile_code[14:0], 5'b00000} + {16'd0, word_in_tile};
        gp1_offset = {1'b0, tile_code[13:0], 5'b00000} + {15'd0, word_in_tile};
        gp_gfx_word_offset_calc = gp_sel ? {{(AW-20){1'b0}}, gp1_offset} :
                                           {{(AW-21){1'b0}}, gp0_offset};
    end
endfunction

function [AW-1:0] gp_gfx_addr_calc;
    input        gp_sel;
    input [15:0] tile_code;
    input [4:0]  word_in_tile;
    input [1:0]  map_mode;
    input        high_half;
    reg [AW-1:0] word_offset;
    reg [AW-1:0] mapped_offset;
    reg [AW-1:0] base_addr;
    reg [AW-1:0] pair_offset;
    reg [AW-1:0] addr_a;
    reg [AW-1:0] addr_b;
    begin
        word_offset = gp_gfx_word_offset_calc(gp_sel, tile_code, word_in_tile);
        mapped_offset = word_offset ^ {{(AW-1){1'b0}}, map_mode[1]};
        base_addr = gp_sel ? 22'h200000 : 22'h000000;
        pair_offset = mapped_offset << 1;
        addr_a = base_addr + pair_offset;
        addr_b = addr_a + {{(AW-1){1'b0}}, 1'b1};
        gp_gfx_addr_calc = high_half ? (map_mode[0] ? addr_a : addr_b) :
                                      (map_mode[0] ? addr_b : addr_a);
    end
endfunction

function [AW-1:0] gp_sprite_gfx_addr_calc;
    input        gp_sel;
    input [17:0] sprite_code;
    input [2:0]  row;
    input [1:0]  map_mode;
    input        high_half;
    reg [AW-1:0] word_offset;
    reg [AW-1:0] mapped_offset;
    reg [AW-1:0] base_addr;
    reg [AW-1:0] pair_offset;
    reg [AW-1:0] addr_a;
    reg [AW-1:0] addr_b;
    begin
        word_offset = gp_sel ? {{(AW-19){1'b0}}, sprite_code[15:0], 3'b000} :
                               {{(AW-20){1'b0}}, sprite_code[16:0], 3'b000};
        word_offset = word_offset + {{(AW-3){1'b0}}, row};
        mapped_offset = word_offset ^ {{(AW-1){1'b0}}, map_mode[1]};
        base_addr = gp_sel ? 22'h200000 : 22'h000000;
        pair_offset = mapped_offset << 1;
        addr_a = base_addr + pair_offset;
        addr_b = addr_a + {{(AW-1){1'b0}}, 1'b1};
        gp_sprite_gfx_addr_calc = high_half ? (map_mode[0] ? addr_a : addr_b) :
                                               (map_mode[0] ? addr_b : addr_a);
    end
endfunction

function [3:0] gp_decode_sample_calc;
    input [15:0] lo_word;
    input [15:0] hi_word;
    input [2:0]  px;
    input [1:0]  decode_mode;
    reg [2:0] bit_msb;
    reg [3:0] idx_msb;
    reg [3:0] idx_msb_hi;
    begin
        bit_msb = 3'd7 - px;
        idx_msb = {1'b0, bit_msb};
        idx_msb_hi = 4'd8 + idx_msb;

        case (decode_mode)
            // Same pixel geometry, different pen bit weights. This isolates
            // metallic/highlight color errors from tile assembly errors.
            2'd1: gp_decode_sample_calc = {
                hi_word[idx_msb_hi],
                hi_word[idx_msb],
                lo_word[idx_msb_hi],
                lo_word[idx_msb]
            };
            2'd2: gp_decode_sample_calc = {
                hi_word[idx_msb],
                hi_word[idx_msb_hi],
                lo_word[idx_msb],
                lo_word[idx_msb_hi]
            };
            2'd3: gp_decode_sample_calc = {
                lo_word[idx_msb_hi],
                lo_word[idx_msb],
                hi_word[idx_msb_hi],
                hi_word[idx_msb]
            };
            default: gp_decode_sample_calc = {
                lo_word[idx_msb],
                lo_word[idx_msb_hi],
                hi_word[idx_msb],
                hi_word[idx_msb_hi]
            };
        endcase
    end
endfunction

function [3:0] gp_sprite_decode_sample_calc;
    input [15:0] lo_word;
    input [15:0] hi_word;
    input [2:0]  px;
    input [1:0]  decode_mode;
    begin
        gp_sprite_decode_sample_calc = gp_decode_sample_calc(lo_word, hi_word,
                                                             px, decode_mode);
    end
endfunction

function [3:0] gp_obj_packed_sample_calc;
    input [15:0] packed_lo;
    input [15:0] packed_hi;
    input [2:0]  px;
    begin
        case (px)
            3'd0: gp_obj_packed_sample_calc = packed_lo[3:0];
            3'd1: gp_obj_packed_sample_calc = packed_lo[7:4];
            3'd2: gp_obj_packed_sample_calc = packed_lo[11:8];
            3'd3: gp_obj_packed_sample_calc = packed_lo[15:12];
            3'd4: gp_obj_packed_sample_calc = packed_hi[3:0];
            3'd5: gp_obj_packed_sample_calc = packed_hi[7:4];
            3'd6: gp_obj_packed_sample_calc = packed_hi[11:8];
            default: gp_obj_packed_sample_calc = packed_hi[15:12];
        endcase
    end
endfunction

function [31:0] gp_obj_word_pixels_calc;
    input [15:0] tile0_lo;
    input [15:0] tile0_hi;
    input [15:0] tile1_lo;
    input [15:0] tile1_hi;
    input [7:0]  valid_mask;
    input [3:0]  source_bias;
    input        flipx;
    input [1:0]  decode_mode;
    integer      out_px;
    reg [3:0]    source_px;
    reg [2:0]    tile_px;
    reg [3:0]    pen;
    begin
        gp_obj_word_pixels_calc = 32'h00000000;
        for (out_px = 0; out_px < 8; out_px = out_px + 1) begin
            source_px = source_bias + out_px;
            tile_px = flipx ? (3'd7 - source_px[2:0]) : source_px[2:0];
            if (valid_mask[out_px]) begin
                pen = source_px[3] ?
                    gp_sprite_decode_sample_calc(tile1_lo, tile1_hi,
                                                 tile_px, decode_mode) :
                    gp_sprite_decode_sample_calc(tile0_lo, tile0_hi,
                                                 tile_px, decode_mode);
            end else begin
                pen = 4'h0;
            end
            gp_obj_word_pixels_calc[(out_px * 4) +: 4] = pen;
        end
    end
endfunction

function gp_raw_any_bit_calc;
    input [15:0] lo_word;
    input [15:0] hi_word;
    input [2:0]  px;
    reg [2:0] bit_lsb;
    reg [2:0] bit_msb;
    reg [3:0] idx_lsb;
    reg [3:0] idx_msb;
    reg [3:0] idx_lsb_hi;
    reg [3:0] idx_msb_hi;
    begin
        bit_lsb = px;
        bit_msb = 3'd7 - px;
        idx_lsb = {1'b0, bit_lsb};
        idx_msb = {1'b0, bit_msb};
        idx_lsb_hi = 4'd8 + idx_lsb;
        idx_msb_hi = 4'd8 + idx_msb;
        gp_raw_any_bit_calc = lo_word[idx_lsb] | lo_word[idx_lsb_hi] |
                              hi_word[idx_lsb] | hi_word[idx_lsb_hi] |
                              lo_word[idx_msb] | lo_word[idx_msb_hi] |
                              hi_word[idx_msb] | hi_word[idx_msb_hi];
    end
endfunction

function [3:0] gp_decode_probe_sample_calc;
    input [15:0] lo_word;
    input [15:0] hi_word;
    input [2:0]  px;
    input [1:0]  decode_mode;
    reg [2:0] bit_lsb;
    reg [2:0] bit_msb;
    reg [3:0] idx_lsb;
    reg [3:0] idx_msb;
    reg [3:0] idx_lsb_hi;
    reg [3:0] idx_msb_hi;
    reg [15:0] lo_bswap;
    reg [15:0] hi_bswap;
    reg [7:0] gfx_a;
    reg [7:0] gfx_b;
    reg [7:0] gfx_c;
    reg [7:0] gfx_d;
    reg [1:0] pair;
    reg [2:0] pair_bit0;
    reg [2:0] pair_bit1;
    reg [2:0] raizing_bit;
    begin
        bit_lsb = px;
        bit_msb = 3'd7 - px;
        idx_lsb = {1'b0, bit_lsb};
        idx_msb = {1'b0, bit_msb};
        idx_lsb_hi = 4'd8 + idx_lsb;
        idx_msb_hi = 4'd8 + idx_msb;
        lo_bswap = {lo_word[7:0], lo_word[15:8]};
        hi_bswap = {hi_word[7:0], hi_word[15:8]};
        gfx_a = lo_word[15:8];
        gfx_b = lo_word[7:0];
        gfx_c = hi_word[15:8];
        gfx_d = hi_word[7:0];
        pair = px[2:1];
        pair_bit0 = 3'd7 - {pair, 1'b0};
        pair_bit1 = pair_bit0 - 3'd1;
        raizing_bit = px[0] ? pair_bit1 : pair_bit0;

        case (decode_mode)
            2'd0: gp_decode_probe_sample_calc = {
                lo_word[idx_lsb],
                lo_word[idx_lsb_hi],
                hi_word[idx_lsb],
                hi_word[idx_lsb_hi]
            };
            2'd1: gp_decode_probe_sample_calc = {
                lo_word[idx_msb],
                lo_word[idx_msb_hi],
                hi_word[idx_msb],
                hi_word[idx_msb_hi]
            };
            2'd2: gp_decode_probe_sample_calc = {
                lo_bswap[idx_lsb],
                lo_bswap[idx_lsb_hi],
                hi_bswap[idx_lsb],
                hi_bswap[idx_lsb_hi]
            };
            default: gp_decode_probe_sample_calc = {
                gfx_d[raizing_bit],
                gfx_b[raizing_bit],
                gfx_c[raizing_bit],
                gfx_a[raizing_bit]
            };
        endcase
    end
endfunction

function [14:0] gp_layer_priority_mux;
    input [14:0] layer0_pixel;
    input [14:0] layer1_pixel;
    input [14:0] layer2_pixel;
    integer pri;
    begin
        gp_layer_priority_mux = 15'h0000;
        for (pri = 0; pri < 16; pri = pri + 1) begin
            if ((layer0_pixel[10:0] != 11'h000) && (layer0_pixel[14:11] == pri[3:0])) begin
                gp_layer_priority_mux = layer0_pixel;
            end
            if ((layer1_pixel[10:0] != 11'h000) && (layer1_pixel[14:11] == pri[3:0])) begin
                gp_layer_priority_mux = layer1_pixel;
            end
            if ((layer2_pixel[10:0] != 11'h000) && (layer2_pixel[14:11] == pri[3:0])) begin
                gp_layer_priority_mux = layer2_pixel;
            end
        end
    end
endfunction

function [1:0] gp_layer_priority_src;
    input [14:0] layer0_pixel;
    input [14:0] layer1_pixel;
    input [14:0] layer2_pixel;
    integer pri;
    begin
        gp_layer_priority_src = 2'd3;
        for (pri = 0; pri < 16; pri = pri + 1) begin
            if ((layer0_pixel[10:0] != 11'h000) && (layer0_pixel[14:11] == pri[3:0])) begin
                gp_layer_priority_src = 2'd0;
            end
            if ((layer1_pixel[10:0] != 11'h000) && (layer1_pixel[14:11] == pri[3:0])) begin
                gp_layer_priority_src = 2'd1;
            end
            if ((layer2_pixel[10:0] != 11'h000) && (layer2_pixel[14:11] == pri[3:0])) begin
                gp_layer_priority_src = 2'd2;
            end
        end
    end
endfunction

function [1:0] comp_fetch_rr_pick3;
    input [2:0] pending;
    input [1:0] start;
    begin
        case (start)
            2'd0: comp_fetch_rr_pick3 = pending[0] ? 2'd0 :
                                           pending[1] ? 2'd1 : 2'd2;
            2'd1: comp_fetch_rr_pick3 = pending[1] ? 2'd1 :
                                           pending[2] ? 2'd2 : 2'd0;
            default: comp_fetch_rr_pick3 = pending[2] ? 2'd2 :
                                               pending[0] ? 2'd0 : 2'd1;
        endcase
    end
endfunction

function [3:0] comp_phase_deadline;
    input [2:0] phase;
    begin
        case (phase)
            3'd0: comp_phase_deadline = 4'd8;
            3'd1: comp_phase_deadline = 4'd7;
            3'd2: comp_phase_deadline = 4'd6;
            3'd3: comp_phase_deadline = 4'd5;
            3'd4: comp_phase_deadline = 4'd4;
            3'd5: comp_phase_deadline = 4'd3;
            3'd6: comp_phase_deadline = 4'd2;
            default: comp_phase_deadline = 4'd1;
        endcase
    end
endfunction

function [1:0] comp_fetch_deadline_pick3;
    input [2:0] pending;
    input [1:0] start;
    input [3:0] deadline0;
    input [3:0] deadline1;
    input [3:0] deadline2;
    reg [1:0] best;
    reg [3:0] best_deadline;
    begin
        best = comp_fetch_rr_pick3(pending, start);
        case (best)
            2'd0: best_deadline = deadline0;
            2'd1: best_deadline = deadline1;
            default: best_deadline = deadline2;
        endcase

        if (pending[0] && (deadline0 < best_deadline)) begin
            best = 2'd0;
            best_deadline = deadline0;
        end
        if (pending[1] && (deadline1 < best_deadline)) begin
            best = 2'd1;
            best_deadline = deadline1;
        end
        if (pending[2] && (deadline2 < best_deadline)) begin
            best = 2'd2;
        end

        comp_fetch_deadline_pick3 = best;
    end
endfunction

function [7:0] gp_obj_size_calc;
    input [3:0] size_bits;
    begin
        gp_obj_size_calc = ({4'h0, size_bits} + 8'd1) << 3;
    end
endfunction

function [7:0] gp_obj_word_mask_calc;
    input [8:0] word_dx;
    input [7:0] width_px;
    integer     mask_px;
    reg [8:0]   pixel_dx;
    begin
        gp_obj_word_mask_calc = 8'h00;
        for (mask_px = 0; mask_px < 8; mask_px = mask_px + 1) begin
            pixel_dx = word_dx + mask_px;
            gp_obj_word_mask_calc[mask_px] =
                pixel_dx < {1'b0, width_px};
        end
    end
endfunction

function [8:0] gp_obj_abs_base_calc;
    input [15:0] pos_word;
    input [15:0] scroll;
    begin
        gp_obj_abs_base_calc = pos_word[15:7] - scroll[8:0];
    end
endfunction

function [8:0] gp_obj_rel_base_calc;
    input [8:0] old_pos;
    input [15:0] pos_word;
    begin
        gp_obj_rel_base_calc = old_pos + pos_word[15:7];
    end
endfunction

function [8:0] gp_obj_x_from_base_calc;
    input [15:0] attr;
    input [8:0]  base_x;
    input [7:0]  width_px;
    reg [8:0] sx_base;
    begin
        sx_base = base_x + 9'h1cc - (attr[12] ? 9'd7 : 9'd0);
        gp_obj_x_from_base_calc = sx_base -
                                  (attr[12] ? ({1'b0, width_px} - 9'd8) : 9'd0);
    end
endfunction

function [8:0] gp_obj_y_from_base_calc;
    input [15:0] attr;
    input [8:0]  base_y;
    input [7:0]  height_px;
    reg [8:0] sy_base;
    begin
        sy_base = base_y + 9'h1ef - (attr[13] ? 9'd7 : 9'd0);
        gp_obj_y_from_base_calc = sy_base -
                                  (attr[13] ? ({1'b0, height_px} - 9'd8) : 9'd0);
    end
endfunction

function [8:0] gp_obj_draw_x_from_raw_calc;
    input [15:0] attr;
    input [8:0]  raw_base_x;
    input [15:0] scrollx;
    input [7:0]  width_px;
    begin
        gp_obj_draw_x_from_raw_calc = gp_obj_x_from_base_calc(
            attr,
            raw_base_x - scrollx[8:0],
            width_px
        );
    end
endfunction

function [8:0] gp_obj_draw_y_from_raw_calc;
    input [15:0] attr;
    input [8:0]  raw_base_y;
    input [15:0] scrolly;
    input [7:0]  height_px;
    begin
        gp_obj_draw_y_from_raw_calc = gp_obj_y_from_base_calc(
            attr,
            raw_base_y - scrolly[8:0],
            height_px
        );
    end
endfunction

function [8:0] gp_obj_x_calc;
    input [15:0] attr;
    input [15:0] xpos_word;
    input [15:0] scrollx;
    begin
        gp_obj_x_calc = gp_obj_x_from_base_calc(
            attr,
            gp_obj_abs_base_calc(xpos_word, scrollx),
            gp_obj_size_calc(xpos_word[3:0])
        );
    end
endfunction

function [8:0] gp_obj_y_calc;
    input [15:0] attr;
    input [15:0] ypos_word;
    input [15:0] scrolly;
    begin
        gp_obj_y_calc = gp_obj_y_from_base_calc(
            attr,
            gp_obj_abs_base_calc(ypos_word, scrolly),
            gp_obj_size_calc(ypos_word[3:0])
        );
    end
endfunction

function [8:0] gp_obj_subtile_index_calc;
    input [4:0] width_tiles;
    input [4:0] tile_x;
    input [4:0] tile_y;
    reg [8:0] tile_x_ext;
    reg [8:0] tile_y_ext;
    reg [8:0] row_base;
    begin
        tile_x_ext = {4'd0, tile_x};
        tile_y_ext = {4'd0, tile_y};
        case (width_tiles)
            5'd1: row_base = tile_y_ext;
            5'd2: row_base = tile_y_ext << 1;
            5'd3: row_base = (tile_y_ext << 1) + tile_y_ext;
            5'd4: row_base = tile_y_ext << 2;
            5'd5: row_base = (tile_y_ext << 2) + tile_y_ext;
            5'd6: row_base = (tile_y_ext << 2) + (tile_y_ext << 1);
            5'd7: row_base = (tile_y_ext << 3) - tile_y_ext;
            5'd8: row_base = tile_y_ext << 3;
            5'd9: row_base = (tile_y_ext << 3) + tile_y_ext;
            5'd10: row_base = (tile_y_ext << 3) + (tile_y_ext << 1);
            5'd11: row_base = (tile_y_ext << 3) + (tile_y_ext << 1) + tile_y_ext;
            5'd12: row_base = (tile_y_ext << 3) + (tile_y_ext << 2);
            5'd13: row_base = (tile_y_ext << 3) + (tile_y_ext << 2) + tile_y_ext;
            5'd14: row_base = (tile_y_ext << 4) - (tile_y_ext << 1);
            5'd15: row_base = (tile_y_ext << 4) - tile_y_ext;
            default: row_base = tile_y_ext << 4;
        endcase
        gp_obj_subtile_index_calc = row_base + tile_x_ext;
    end
endfunction

function [14:0] gp_obj_silhouette_pixel;
    input [15:0] attr;
    begin
        gp_obj_silhouette_pixel = {attr[11:8], 1'b0, attr[7:2], 4'hf};
    end
endfunction

function gp_obj_entry_live_calc;
    input [15:0] attr;
    input [15:0] code_word;
    input [15:0] xpos_word;
    input [15:0] ypos_word;
    begin
        gp_obj_entry_live_calc = attr[15] &&
            !((attr == 16'hc000) && (code_word == 16'h00c0) &&
              (xpos_word == 16'hc000) && (ypos_word == 16'h0000));
    end
endfunction

function gp_obj_axis_visible_calc;
    input [8:0] pos;
    input [7:0] size_px;
    input [8:0] limit_px;
    reg [9:0] span_end;
    begin
        span_end = {1'b0, pos} + {2'b00, size_px};
        gp_obj_axis_visible_calc = (pos < limit_px) || (span_end > 10'd512);
    end
endfunction

function gp_obj_rect_visible_calc;
    input [8:0] x;
    input [8:0] y;
    input [7:0] w;
    input [7:0] h;
    begin
        gp_obj_rect_visible_calc =
            gp_obj_axis_visible_calc(x, w, 9'd320) &&
            gp_obj_axis_visible_calc(y, h, 9'd240);
    end
endfunction

reg [3:0] clkdiv = 4'd0;
reg [9:0] hcnt = 10'd0;
reg [8:0] vcnt = 9'd0;
wire render_lhbl;
wire render_lvbl;
wire debug_frame_tick = (clkdiv == 4'd13) &&
                        (hcnt == (H_TOTAL - 10'd1)) &&
                        (vcnt == (V_TOTAL - 9'd1));
wire debug_line_tick = (clkdiv == 4'd13) &&
                       (hcnt == (H_TOTAL - 10'd1));
wire [15:0] rom_slot_dout;
wire        rom_slot_ok;
wire        rom_slot_rd;
wire [17:0] rom_slot_addr;
wire        rom_slot_cs;
wire [15:0] gfx_scroll_slot_dout;
wire [15:0] gfx_obj_slot_dout;
wire [15:0] gfx_probe_slot_dout;
wire        gfx_scroll_slot_ok;
wire        gfx_obj_slot_ok;
wire        gfx_probe_slot_ok;
wire        gfx_slot_rd;
wire [AW-1:0] gfx_scroll_slot_addr;
wire [AW-1:0] gfx_obj_slot_addr;
wire [AW-1:0] gfx_probe_slot_addr;
wire        gfx_scroll_slot_cs;
wire        gfx_obj_slot_cs;
wire        gfx_probe_slot_cs;
wire [AW-1:0] gfx_sdram_addr;
`ifdef BATSUGUN_HW_DEBUG
wire [31:0]  video_profile_source;
wire [127:0] video_profile_probe;
wire [127:0] video_profile_profiler_probe;
wire         video_profile_gfx_reset = video_profile_source[6];
wire         video_profile_mem_probe = video_profile_source[7];
wire [AW-1:0] video_profile_mem_addr = video_profile_source[8 +: AW];
wire [1:0]   obj_debug_buffer_mode = video_profile_source[31:30];
wire         sound_diag_enable = video_profile_source[27];
wire         sound_diag_hold_v25 = video_profile_source[26];
wire         sound_diag_block_ym = video_profile_source[25];
wire         sound_diag_block_oki = video_profile_source[24];
wire         sound_diag_core_reset = video_profile_source[23];
wire         sound_diag_cpu_slow = video_profile_source[22];
wire         sound_diag_cpu_wait2 = video_profile_source[21];
wire         sound_diag_cpu_wait3 = video_profile_source[20];
`else
wire         video_profile_gfx_reset = 1'b0;
wire         video_profile_mem_probe = 1'b0;
wire [AW-1:0] video_profile_mem_addr = {AW{1'b0}};
wire [1:0]   obj_debug_buffer_mode = 2'd0;
wire         sound_diag_enable = 1'b0;
wire         sound_diag_hold_v25 = 1'b0;
wire         sound_diag_block_ym = 1'b0;
wire         sound_diag_block_oki = 1'b0;
wire         sound_diag_core_reset = 1'b0;
wire         sound_diag_cpu_slow = 1'b0;
wire         sound_diag_cpu_wait2 = 1'b0;
wire         sound_diag_cpu_wait3 = 1'b0;
`endif
reg [19:0] gfx_startup_hold = 20'hfffff;
wire gfx_startup_reset = |gfx_startup_hold;
wire rom_runtime_reset = rst || dwnld_busy || ioctl_rom ||
                         gfx_startup_reset;
wire video_runtime_reset = rom_runtime_reset || ss_video_reset;
reg  [AW-1:0] gfx_req_addr = {AW{1'b0}};
reg  [AW-1:0] gfx_req_high_addr = {AW{1'b0}};
reg  [8:0] gfx_req_target_x = 9'd0;
reg  [8:0] gfx_req_target_y = 9'd0;
reg        gfx_req_pending = 1'b0;
reg        gfx_req_phase = 1'b0;
reg        gfx_req_stage = 1'b0;
reg        gfx_req_far_stage = 1'b0;
reg        gfx_req_deep_stage = 1'b0;
reg        gfx_req_valid = 1'b0;
reg  [6:0] gfx_req_color = 7'h00;
reg  [3:0] gfx_req_pri = 4'h0;
reg  [15:0] gfx_fetch_lo = 16'h0000;
reg  [AW-1:0] gfx_req_probe_addr = {AW{1'b0}};
reg  [AW-1:0] gfx_req_probe_high_addr = {AW{1'b0}};
reg  [8:0] gfx_req_probe_target_x = 9'd0;
reg  [8:0] gfx_req_probe_target_y = 9'd0;
reg        gfx_req_probe_pending = 1'b0;
reg        gfx_req_probe_phase = 1'b0;
reg        gfx_req_probe_stage = 1'b0;
reg        gfx_req_probe_far_stage = 1'b0;
reg        gfx_req_probe_deep_stage = 1'b0;
reg        gfx_req_probe_valid = 1'b0;
reg  [6:0] gfx_req_probe_color = 7'h00;
reg  [3:0] gfx_req_probe_pri = 4'h0;
reg  [15:0] gfx_fetch_probe_lo = 16'h0000;
reg        gfx_seen_ok = 1'b0;
reg  [12:0] gp0_comp_scan_addr = 13'h0000;
reg  [12:0] gp1_comp_scan_addr = 13'h0000;
reg  [15:0] comp_tile_attr [0:5];
reg  [15:0] comp_tile_code [0:5];
reg  [AW-1:0] comp_fetch_addr_lo [0:5];
reg  [AW-1:0] comp_fetch_addr_hi [0:5];
reg  [8:0] comp_fetch_target_x [0:5];
reg  [8:0] comp_fetch_target_y [0:5];
reg  [6:0] comp_fetch_color [0:5];
reg  [3:0] comp_fetch_pri [0:5];
reg  [5:0] comp_fetch_pending = 6'b000000;
reg  [5:0] comp_fetch_stage = 6'b000000;
reg  [5:0] comp_fetch_far_stage = 6'b000000;
reg  [5:0] comp_fetch_deep_stage = 6'b000000;
reg  [5:0] comp_word_start_event = 6'b000000;
// Four tagged words per layer cover the 16-pixel lookahead without relying
// on relative stage shifts across inactive scan regions.
reg  [15:0] comp_word_cache_lo [0:23];
reg  [15:0] comp_word_cache_hi [0:23];
reg  [6:0] comp_word_cache_color [0:23];
reg  [3:0] comp_word_cache_pri [0:23];
reg  [8:0] comp_word_cache_target_x [0:23];
reg  [8:0] comp_word_cache_target_y [0:23];
reg        comp_word_cache_valid [0:23];
reg        comp_word_cache_ready [0:23];
reg  [15:0] comp_latched_lo [0:5];
reg  [15:0] comp_latched_hi [0:5];
reg  [6:0] comp_latched_color [0:5];
reg  [3:0] comp_latched_pri [0:5];
reg  [5:0] comp_latched_valid = 6'b000000;
reg  [2:0] gfx_req_slot = 3'd0;
reg  [2:0] gfx_req_probe_slot = 3'd3;
reg  [2:0] comp_fetch_rr = 3'd0;
reg  [2:0] comp_fetch_probe_rr = 3'd3;
reg  [2:0] comp_fetch_grant_slot = 3'd0;
reg        comp_fetch_grant_valid = 1'b0;
reg  [2:0] comp_fetch_probe_grant_slot = 3'd3;
reg        comp_fetch_probe_grant_valid = 1'b0;
reg        comp_frame_bank = 1'b0;
integer comp_i;
integer comp_cache_i;
localparam OBJ_BOX_COUNT = 96;
localparam OBJ_DEBUG_COUNT = 16;
localparam OBJ_CACHE_ADDR_W = 7;
localparam OBJ_CACHE_DATA_W = 68;
`ifdef SIMULATION
localparam OBJ_AUX_COUNT = OBJ_BOX_COUNT;
`else
localparam OBJ_AUX_COUNT = OBJ_DEBUG_COUNT;
`endif
localparam OBJ_LINE_EXTRA_PREFETCH_SLOTS = 9;
localparam [3:0] OBJ_LINE_COMMIT_NONE = 4'd15;
localparam [8:0] OBJ_SEQ_LOOKAHEAD = 9'd48;
localparam [8:0] OBJ_REQ_URGENT_PIXELS = 9'd8;
localparam [12:0] GP_OBJ_BASE = 13'h1800;
reg  [7:0] obj_scan_idx = 8'h00;
reg  [3:0] obj_scan_phase = 4'd0;
reg  [15:0] gp0_obj_word0 = 16'h0000;
reg  [15:0] gp0_obj_word1 = 16'h0000;
reg  [15:0] gp0_obj_word2 = 16'h0000;
reg  [15:0] gp0_obj_word3 = 16'h0000;
reg  [15:0] gp1_obj_word0 = 16'h0000;
reg  [15:0] gp1_obj_word1 = 16'h0000;
reg  [15:0] gp1_obj_word2 = 16'h0000;
reg  [15:0] gp1_obj_word3 = 16'h0000;
reg  [8:0] gp0_obj_old_x = 9'd0;
reg  [8:0] gp0_obj_old_y = 9'd0;
reg  [8:0] gp1_obj_old_x = 9'd0;
reg  [8:0] gp1_obj_old_y = 9'd0;
reg        gp0_obj_stage_live = 1'b0;
reg        gp1_obj_stage_live = 1'b0;
reg  [8:0] gp0_obj_stage_base_x = 9'd0;
reg  [8:0] gp0_obj_stage_base_y = 9'd0;
reg  [8:0] gp1_obj_stage_base_x = 9'd0;
reg  [8:0] gp1_obj_stage_base_y = 9'd0;
reg  [8:0] gp0_obj_stage_draw_x = 9'd0;
reg  [8:0] gp0_obj_stage_draw_y = 9'd0;
reg  [8:0] gp1_obj_stage_draw_x = 9'd0;
reg  [8:0] gp1_obj_stage_draw_y = 9'd0;
reg  [7:0] gp0_obj_stage_w = 8'd0;
reg  [7:0] gp0_obj_stage_h = 8'd0;
reg  [7:0] gp1_obj_stage_w = 8'd0;
reg  [7:0] gp1_obj_stage_h = 8'd0;
reg  [6:0] gp0_obj_count = 7'd0;
reg  [6:0] gp1_obj_count = 7'd0;
// The production line renderer reads a packed synchronous cache below. Keep
// only the first few objects in flops for the interactive diagnostic views.
reg  [8:0] gp0_obj_x [0:OBJ_AUX_COUNT-1];
reg  [8:0] gp0_obj_y [0:OBJ_AUX_COUNT-1];
reg  [8:0] gp0_obj_raw_base_x [0:OBJ_AUX_COUNT-1];
reg  [8:0] gp0_obj_raw_base_y [0:OBJ_AUX_COUNT-1];
reg  [7:0] gp0_obj_w [0:OBJ_AUX_COUNT-1];
reg  [7:0] gp0_obj_h [0:OBJ_AUX_COUNT-1];
reg  [15:0] gp0_obj_attr [0:OBJ_AUX_COUNT-1];
reg  [17:0] gp0_obj_code [0:OBJ_AUX_COUNT-1];
reg  [15:0] gp0_obj_raw_x [0:OBJ_AUX_COUNT-1];
reg  [15:0] gp0_obj_raw_y [0:OBJ_AUX_COUNT-1];
reg  [8:0] gp1_obj_x [0:OBJ_AUX_COUNT-1];
reg  [8:0] gp1_obj_y [0:OBJ_AUX_COUNT-1];
reg  [8:0] gp1_obj_raw_base_x [0:OBJ_AUX_COUNT-1];
reg  [8:0] gp1_obj_raw_base_y [0:OBJ_AUX_COUNT-1];
reg  [7:0] gp1_obj_w [0:OBJ_AUX_COUNT-1];
reg  [7:0] gp1_obj_h [0:OBJ_AUX_COUNT-1];
reg  [15:0] gp1_obj_attr [0:OBJ_AUX_COUNT-1];
reg  [17:0] gp1_obj_code [0:OBJ_AUX_COUNT-1];
reg  [15:0] gp1_obj_raw_x [0:OBJ_AUX_COUNT-1];
reg  [15:0] gp1_obj_raw_y [0:OBJ_AUX_COUNT-1];
`ifdef BATSUGUN_HW_DEBUG
reg  [19:0] obj_trace_frame = 20'h00000;
reg  [15:0] gp0_obj_trace_attr3 = 16'h0000;
reg  [15:0] gp0_obj_trace_code3 = 16'h0000;
reg  [8:0]  gp0_obj_trace_base_x3 = 9'h000;
reg  [8:0]  gp0_obj_trace_base_y3 = 9'h000;
reg  [8:0]  gp0_obj_trace_draw_x3 = 9'h000;
reg  [8:0]  gp0_obj_trace_draw_y3 = 9'h000;
reg  [15:0] gp0_obj_trace_attr4 = 16'h0000;
reg  [15:0] gp0_obj_trace_code4 = 16'h0000;
reg  [8:0]  gp0_obj_trace_base_x4 = 9'h000;
reg  [8:0]  gp0_obj_trace_base_y4 = 9'h000;
reg  [8:0]  gp0_obj_trace_draw_x4 = 9'h000;
reg  [8:0]  gp0_obj_trace_draw_y4 = 9'h000;
reg  [15:0] gp0_obj_trace_attr5 = 16'h0000;
reg  [15:0] gp0_obj_trace_code5 = 16'h0000;
reg  [8:0]  gp0_obj_trace_base_x5 = 9'h000;
reg  [8:0]  gp0_obj_trace_base_y5 = 9'h000;
reg  [8:0]  gp0_obj_trace_draw_x5 = 9'h000;
reg  [8:0]  gp0_obj_trace_draw_y5 = 9'h000;
`endif
reg        gp0_obj_box_bit;
reg        gp1_obj_box_bit;
reg        gp0_obj_fill_bit;
reg        gp1_obj_fill_bit;
reg  [3:0] gp0_obj_sprite_sample;
reg  [3:0] gp1_obj_sprite_sample;
reg        gp0_obj_sprite_raw_bit;
reg        gp1_obj_sprite_raw_bit;
reg  [3:0] gp0_obj_sprite_pri;
reg  [3:0] gp1_obj_sprite_pri;
reg  [14:0] gp0_obj_fill_pixel;
reg  [14:0] gp1_obj_fill_pixel;
reg  [8:0] obj_box_dx;
reg  [8:0] obj_box_dy;
reg  [2:0] obj_sprite_x;
reg  [2:0] obj_sprite_y;
reg  [4:0] obj_box_idx;
reg  [8:0] obj_line_fetch_x;
reg  [8:0] obj_line_dx;
reg  [8:0] obj_line_dy;
reg  [4:0] obj_line_w_tiles;
reg  [4:0] obj_line_h_tiles;
reg  [4:0] obj_line_tile_x;
reg  [4:0] obj_line_tile_y;
reg  [8:0] obj_line_tile_index;
reg        obj_line_pick_valid;
reg        obj_line_pick_gp_sel;
reg  [17:0] obj_line_pick_code;
reg  [2:0] obj_line_pick_row;
reg  [6:0] obj_line_pick_color;
reg  [3:0] obj_line_pick_pri;
reg        obj_line_pick_flipx;
integer obj_reset_i;
integer obj_box_i;
reg  [4:0] obj_gfx_fetch_slot = 5'd0;
reg  [7:0] obj_gfx_fetch_tile = 8'd0;
reg  [2:0] obj_gfx_fetch_row = 3'd0;
reg  [1:0] obj_gfx_fetch_phase = 2'd0;
reg        obj_gfx_fetch_done = 1'b0;
reg        obj_gfx_req_pending = 1'b0;
reg  [AW-1:0] obj_gfx_req_addr = {AW{1'b0}};
reg  [15:0] obj_gfx_fetch_lo = 16'h0000;
reg  [12:0] obj_gfx_valid_count = 13'd0;
reg  [12:0] obj_gfx_nonzero_count = 13'd0;
reg        obj_gfx_seen_ok = 1'b0;
reg        obj_gfx_seen_nonzero = 1'b0;
reg        obj_gfx_debug_slot_valid = 1'b0;
reg  [4:0] obj_gfx_debug_slot = 5'd0;
reg  [7:0] obj_gfx_debug_tile = 8'd0;
reg        obj_gfx_debug_hit_valid = 1'b0;
reg  [2:0] obj_gfx_debug_hit_row = 3'd0;
reg  [15:0] obj_gfx_debug_hit_lo = 16'h0000;
reg  [15:0] obj_gfx_debug_hit_hi = 16'h0000;
reg  [15:0] obj_gfx_debug_lo [0:7];
reg  [15:0] obj_gfx_debug_hi [0:7];
reg  [7:0]  obj_gfx_debug_valid = 8'h00;
reg  [AW-1:0] obj_line_req_addr = {AW{1'b0}};
reg  [AW-1:0] obj_line_req_high_addr = {AW{1'b0}};
reg  [AW-1:0] obj_line_req_next_addr = {AW{1'b0}};
reg  [AW-1:0] obj_line_req_next_high_addr = {AW{1'b0}};
reg  [AW-1:0] obj_line_req_alt_addr = {AW{1'b0}};
reg  [AW-1:0] obj_line_req_alt_high_addr = {AW{1'b0}};
reg  [AW-1:0] obj_line_req_alt_next_addr = {AW{1'b0}};
reg  [AW-1:0] obj_line_req_alt_next_high_addr = {AW{1'b0}};
reg        obj_line_req_pending = 1'b0;
reg  [2:0] obj_line_req_phase = 3'd0;
reg        obj_line_req_valid = 1'b0;
reg        obj_line_req_gp_sel = 1'b0;
reg        obj_line_req_flipx = 1'b0;
reg        obj_line_req_next_valid = 1'b0;
reg  [7:0] obj_line_req_mask = 8'h00;
reg  [3:0] obj_line_req_bias = 4'h0;
reg  [6:0] obj_line_req_color = 7'h00;
reg  [3:0] obj_line_req_pri = 4'h0;
reg        obj_line_req_alt_valid = 1'b0;
reg        obj_line_req_alt_gp_sel = 1'b0;
reg        obj_line_req_alt_flipx = 1'b0;
reg        obj_line_req_alt_next_valid = 1'b0;
reg  [7:0] obj_line_req_alt_mask = 8'h00;
reg  [3:0] obj_line_req_alt_bias = 4'h0;
reg  [6:0] obj_line_req_alt_color = 7'h00;
reg  [3:0] obj_line_req_alt_pri = 4'h0;
reg  [8:0] obj_line_req_target_x = 9'd0;
reg  [8:0] obj_line_req_target_y = 9'd0;
reg        obj_line_req_primary_committed = 1'b0;
reg  [3:0] obj_line_req_commit_slot = OBJ_LINE_COMMIT_NONE;
reg  [15:0] obj_line_fetch_lo = 16'h0000;
reg  [15:0] obj_line_tile0_lo = 16'h0000;
reg  [15:0] obj_line_tile0_hi = 16'h0000;
reg  [15:0] obj_line_primary_lo = 16'h0000;
reg  [15:0] obj_line_primary_hi = 16'h0000;
reg  [15:0] obj_line_alt_lo = 16'h0000;
reg  [15:0] obj_line_alt_hi = 16'h0000;
reg         obj_line_primary_commit_pending = 1'b0;
reg         obj_line_alt_commit_pending = 1'b0;
reg  [15:0] obj_line_prefetch_lo = 16'h0000;
reg  [15:0] obj_line_prefetch_hi = 16'h0000;
reg  [15:0] obj_line_prefetch_alt_lo = 16'h0000;
reg  [15:0] obj_line_prefetch_alt_hi = 16'h0000;
reg  [6:0] obj_line_prefetch_color = 7'h00;
reg  [3:0] obj_line_prefetch_pri = 4'h0;
reg  [6:0] obj_line_prefetch_alt_color = 7'h00;
reg  [3:0] obj_line_prefetch_alt_pri = 4'h0;
reg  [8:0] obj_line_prefetch_x = 9'd0;
reg  [8:0] obj_line_prefetch_y = 9'd0;
reg        obj_line_prefetch_valid = 1'b0;
reg        obj_line_prefetch_alt_valid = 1'b0;
reg        obj_line_prefetch_ready = 1'b0;
reg        obj_line_prefetch_gp_sel = 1'b0;
reg        obj_line_prefetch_flipx = 1'b0;
reg        obj_line_prefetch_alt_gp_sel = 1'b0;
reg        obj_line_prefetch_alt_flipx = 1'b0;
reg  [15:0] obj_line_prefetch1_lo = 16'h0000;
reg  [15:0] obj_line_prefetch1_hi = 16'h0000;
reg  [15:0] obj_line_prefetch1_alt_lo = 16'h0000;
reg  [15:0] obj_line_prefetch1_alt_hi = 16'h0000;
reg  [6:0] obj_line_prefetch1_color = 7'h00;
reg  [3:0] obj_line_prefetch1_pri = 4'h0;
reg  [6:0] obj_line_prefetch1_alt_color = 7'h00;
reg  [3:0] obj_line_prefetch1_alt_pri = 4'h0;
reg  [8:0] obj_line_prefetch1_x = 9'd0;
reg  [8:0] obj_line_prefetch1_y = 9'd0;
reg        obj_line_prefetch1_valid = 1'b0;
reg        obj_line_prefetch1_alt_valid = 1'b0;
reg        obj_line_prefetch1_ready = 1'b0;
reg        obj_line_prefetch1_gp_sel = 1'b0;
reg        obj_line_prefetch1_flipx = 1'b0;
reg        obj_line_prefetch1_alt_gp_sel = 1'b0;
reg        obj_line_prefetch1_alt_flipx = 1'b0;
reg  [15:0] obj_line_prefetch2_lo = 16'h0000;
reg  [15:0] obj_line_prefetch2_hi = 16'h0000;
reg  [15:0] obj_line_prefetch2_alt_lo = 16'h0000;
reg  [15:0] obj_line_prefetch2_alt_hi = 16'h0000;
reg  [6:0] obj_line_prefetch2_color = 7'h00;
reg  [3:0] obj_line_prefetch2_pri = 4'h0;
reg  [6:0] obj_line_prefetch2_alt_color = 7'h00;
reg  [3:0] obj_line_prefetch2_alt_pri = 4'h0;
reg  [8:0] obj_line_prefetch2_x = 9'd0;
reg  [8:0] obj_line_prefetch2_y = 9'd0;
reg        obj_line_prefetch2_valid = 1'b0;
reg        obj_line_prefetch2_alt_valid = 1'b0;
reg        obj_line_prefetch2_ready = 1'b0;
reg        obj_line_prefetch2_gp_sel = 1'b0;
reg        obj_line_prefetch2_flipx = 1'b0;
reg        obj_line_prefetch2_alt_gp_sel = 1'b0;
reg        obj_line_prefetch2_alt_flipx = 1'b0;
reg  [15:0] obj_line_prefetch_extra_lo [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg  [15:0] obj_line_prefetch_extra_hi [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg  [15:0] obj_line_prefetch_extra_alt_lo [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg  [15:0] obj_line_prefetch_extra_alt_hi [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg  [6:0] obj_line_prefetch_extra_color [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg  [3:0] obj_line_prefetch_extra_pri [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg  [6:0] obj_line_prefetch_extra_alt_color [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg  [3:0] obj_line_prefetch_extra_alt_pri [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg  [8:0] obj_line_prefetch_extra_x [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg  [8:0] obj_line_prefetch_extra_y [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg        obj_line_prefetch_extra_valid [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg        obj_line_prefetch_extra_alt_valid [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg        obj_line_prefetch_extra_ready [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg        obj_line_prefetch_extra_gp_sel [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg        obj_line_prefetch_extra_flipx [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg        obj_line_prefetch_extra_alt_gp_sel [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
reg        obj_line_prefetch_extra_alt_flipx [0:OBJ_LINE_EXTRA_PREFETCH_SLOTS-1];
integer    obj_line_extra_comb_i;
integer    obj_line_extra_seq_i;
integer    obj_line_event_i;
reg        obj_line_extra_match;
reg  [3:0] obj_line_extra_match_idx;
reg        obj_line_extra_free;
reg  [3:0] obj_line_extra_free_idx;
reg        obj_line_sched_word_event = 1'b0;
reg        obj_line_word_event = 1'b0;
reg        obj_line_prefetch_match_event = 1'b0;
reg        obj_line_prefetch1_match_event = 1'b0;
reg        obj_line_prefetch2_match_event = 1'b0;
reg        obj_line_extra_match_event = 1'b0;
reg  [3:0] obj_line_extra_match_idx_event = 4'd0;
reg        obj_line_prefetch_expired_event = 1'b0;
reg        obj_line_prefetch1_expired_event = 1'b0;
reg        obj_line_prefetch2_expired_event = 1'b0;
reg  [OBJ_LINE_EXTRA_PREFETCH_SLOTS-1:0] obj_line_extra_expired_event =
    {OBJ_LINE_EXTRA_PREFETCH_SLOTS{1'b0}};
reg  [15:0] obj_line_latched_lo = 16'h0000;
reg  [15:0] obj_line_latched_hi = 16'h0000;
reg  [15:0] obj_line_latched_alt_lo = 16'h0000;
reg  [15:0] obj_line_latched_alt_hi = 16'h0000;
reg  [6:0] obj_line_latched_color = 7'h00;
reg  [3:0] obj_line_latched_pri = 4'h0;
reg  [6:0] obj_line_latched_alt_color = 7'h00;
reg  [3:0] obj_line_latched_alt_pri = 4'h0;
reg        obj_line_latched_valid = 1'b0;
reg        obj_line_latched_alt_valid = 1'b0;
reg        obj_line_latched_gp_sel = 1'b0;
reg        obj_line_latched_flipx = 1'b0;
reg        obj_line_latched_alt_gp_sel = 1'b0;
reg        obj_line_latched_alt_flipx = 1'b0;
reg        obj_line_seen_ok = 1'b0;
reg        obj_line_seen_nonzero = 1'b0;
reg        obj_seq_scan_active = 1'b0;
reg  [5:0] obj_seq_scan_idx = 6'd0;
reg  [8:0] obj_seq_target_x = 9'd0;
reg  [8:0] obj_seq_target_y = 9'd0;
reg        obj_seq_pick_ready = 1'b0;
reg        obj_seq_pick_valid = 1'b0;
reg        obj_seq_pick_gp_sel = 1'b0;
reg  [17:0] obj_seq_pick_code = 18'h00000;
reg  [17:0] obj_seq_pick_next_code = 18'h00000;
reg        obj_seq_pick_next_valid = 1'b0;
reg  [7:0] obj_seq_pick_mask = 8'h00;
reg  [3:0] obj_seq_pick_bias = 4'h0;
reg  [2:0] obj_seq_pick_row = 3'd0;
reg  [6:0] obj_seq_pick_color = 7'h00;
reg  [3:0] obj_seq_pick_pri = 4'h0;
reg        obj_seq_pick_flipx = 1'b0;
reg        obj_seq_pick2_valid = 1'b0;
reg        obj_seq_pick2_gp_sel = 1'b0;
reg  [17:0] obj_seq_pick2_code = 18'h00000;
reg  [17:0] obj_seq_pick2_next_code = 18'h00000;
reg        obj_seq_pick2_next_valid = 1'b0;
reg  [7:0] obj_seq_pick2_mask = 8'h00;
reg  [3:0] obj_seq_pick2_bias = 4'h0;
reg  [2:0] obj_seq_pick2_row = 3'd0;
reg  [6:0] obj_seq_pick2_color = 7'h00;
reg  [3:0] obj_seq_pick2_pri = 4'h0;
reg        obj_seq_pick2_flipx = 1'b0;
reg  [8:0] obj_seq_pick_target_x = 9'd0;
reg  [8:0] obj_seq_pick_target_y = 9'd0;
reg        obj_seq_eval_valid = 1'b0;
reg        obj_seq_eval_gp_sel = 1'b0;
reg  [15:0] obj_seq_eval_attr = 16'h0000;
reg  [17:0] obj_seq_eval_code_base = 18'h00000;
reg  [8:0] obj_seq_eval_dx = 9'd0;
reg  [8:0] obj_seq_eval_dy = 9'd0;
reg  [4:0] obj_seq_eval_w_tiles = 5'd0;
reg  [4:0] obj_seq_eval_h_tiles = 5'd0;
reg        obj_seq_scan_done_pending = 1'b0;

// Build each GP9001 object layer ahead of display. Resolving priority per
// opaque pixel preserves lower objects through transparent pixels, unlike
// the legacy two-candidate-per-word prefetch path.
localparam [3:0] OBJ_LB_IDLE        = 4'd0;
localparam [3:0] OBJ_LB_CLEAR       = 4'd1;
localparam [3:0] OBJ_LB_OBJECT      = 4'd2;
localparam [3:0] OBJ_LB_TILE        = 4'd3;
localparam [3:0] OBJ_LB_FETCH_LO    = 4'd4;
localparam [3:0] OBJ_LB_FETCH_HI    = 4'd5;
localparam [3:0] OBJ_LB_PIXEL_READ  = 4'd6;
localparam [3:0] OBJ_LB_PIXEL_WRITE = 4'd7;
localparam [3:0] OBJ_LB_CACHE_WAIT  = 4'd8;
localparam [3:0] OBJ_LB_LATCH_LO    = 4'd9;
localparam [3:0] OBJ_LB_LATCH_HI    = 4'd10;
localparam [3:0] OBJ_LB_CACHE_READ  = 4'd11;
localparam integer OBJ_LB_BANK_BITS = 6;
localparam integer OBJ_LB_BANK_COUNT = 1 << OBJ_LB_BANK_BITS;
localparam integer OBJ_LB_ADDR_BITS = OBJ_LB_BANK_BITS + 9;
reg  [3:0] obj_lb_state = OBJ_LB_IDLE;
reg  [8:0] obj_lb_target_y = 9'd0;
reg  [OBJ_LB_BANK_BITS-1:0] obj_lb_build_bank = {OBJ_LB_BANK_BITS{1'b0}};
reg        obj_lb_frame_active = 1'b0;
reg        obj_lb_cache_scan_active_d = 1'b0;
reg        obj_lb_gp_sel = 1'b0;
reg  [6:0] obj_lb_slot = 7'd0;
reg  [8:0] obj_lb_clear_x = 9'd0;
reg  [8:0] obj_lb_obj_x = 9'd0;
reg  [7:0] obj_lb_obj_w = 8'd0;
reg  [7:0] obj_lb_obj_h = 8'd0;
reg [15:0] obj_lb_obj_attr = 16'h0000;
reg [17:0] obj_lb_obj_code = 18'h00000;
reg  [4:0] obj_lb_w_tiles = 5'd0;
reg  [4:0] obj_lb_tile_x = 5'd0;
reg  [4:0] obj_lb_tile_y = 5'd0;
reg  [2:0] obj_lb_row = 3'd0;
reg  [2:0] obj_lb_pixel = 3'd0;
reg [15:0] obj_lb_tile_lo = 16'h0000;
reg [15:0] obj_lb_tile_hi = 16'h0000;
reg [AW-1:0] obj_lb_req_addr = {AW{1'b0}};
reg [AW-1:0] obj_lb_req_high_addr = {AW{1'b0}};
reg        obj_lb_req_pending = 1'b0;
reg        obj_lb_req_hold = 1'b0;
reg  [OBJ_LB_BANK_COUNT-1:0] obj_lb_bank_ready =
    {OBJ_LB_BANK_COUNT{1'b0}};
reg  [3:0] obj_lb_bank_epoch [0:OBJ_LB_BANK_COUNT-1];
reg  [3:0] obj_lb_build_epoch = 4'd0;
reg        obj_lb_epochs_initialized = 1'b0;
reg  [8:0] obj_lb_bank_y [0:OBJ_LB_BANK_COUNT-1];
reg [12:0] obj_lb_line_cycles = 13'd0;
reg [12:0] obj_lb_last_cycles = 13'd0;
reg [12:0] obj_lb_max_cycles = 13'd0;
reg  [7:0] obj_lb_deadline_miss_count = 8'd0;
reg [12:0] obj_lb_max_cycles_latched = 13'd0;
reg  [7:0] obj_lb_deadline_miss_latched = 8'd0;
integer obj_lb_bank_i;
wire [19:0] gp0_obj_lb_build_q;
wire [19:0] gp1_obj_lb_build_q;
wire [19:0] gp0_obj_lb_display_q;
wire [19:0] gp1_obj_lb_display_q;
wire [OBJ_CACHE_DATA_W-1:0] gp0_obj_cache_q;
wire [OBJ_CACHE_DATA_W-1:0] gp1_obj_cache_q;
reg  [AW-1:0] gfx_fixed_probe_addr = {AW{1'b0}};
reg        gfx_fixed_probe_cs = 1'b0;
reg  [2:0] gfx_fixed_probe_idx = 3'd0;
reg  [5:0] gfx_fixed_probe_seen = 6'b000000;
reg  [15:0] gfx_fixed_probe_word0 = 16'h0000;
reg  [15:0] gfx_fixed_probe_word1 = 16'h0000;
reg  [15:0] gfx_fixed_probe_word2 = 16'h0000;
reg  [15:0] gfx_fixed_probe_word3 = 16'h0000;
reg  [15:0] gfx_fixed_probe_word4 = 16'h0000;
reg  [15:0] gfx_fixed_probe_word5 = 16'h0000;
reg  [1:0] rom_probe_addr = 2'd0;
reg        rom_probe_cs = 1'b0;
reg  [1:0] rom_probe_idx = 2'd0;
reg  [15:0] rom_probe_word0 = 16'h0000;
reg  [15:0] rom_probe_word1 = 16'h0000;
reg  [15:0] rom_probe_word2 = 16'h0000;
reg  [15:0] rom_probe_word3 = 16'h0000;
reg  [7:0] rom_probe_seen = 8'h00;
reg  [15:0] rom_probe_display_word;
reg  [15:0] debug_hold0 = 16'h0000;
reg  [15:0] debug_hold1 = 16'h0000;
reg  [15:0] debug_hold2 = 16'h0000;
reg  [15:0] debug_hold3 = 16'h0000;
reg  [15:0] debug_hold4 = 16'h0000;
reg  [15:0] debug_hold5 = 16'h0000;
reg  [15:0] debug_hold6 = 16'h0000;
reg  [15:0] debug_hold7 = 16'h0000;
reg  [5:0]  debug_hold_div = 6'd0;
reg  [15:0] debug_live0_px = 16'h0000;
reg  [15:0] debug_live1_px = 16'h0000;
reg  [15:0] debug_live2_px = 16'h0000;
reg  [15:0] debug_live3_px = 16'h0000;
reg  [15:0] debug_live4_px = 16'h0000;
reg  [15:0] debug_live5_px = 16'h0000;
reg  [15:0] debug_live6_px = 16'h0000;
reg  [15:0] debug_live7_px = 16'h0000;
reg  [7:0]  comp_miss_count [0:5];
reg  [7:0]  comp_miss_latched [0:5];
reg  [7:0]  comp_miss_peak [0:5];
reg  [7:0]  comp_miss_display [0:5];
reg  [5:0]  comp_miss_line_seen = 6'b000000;
reg  [7:0]  obj_prefetch_miss_count = 8'h00;
reg  [7:0]  obj_prefetch_miss_latched = 8'h00;
reg  [7:0]  obj_prefetch_miss_peak = 8'h00;
reg  [7:0]  obj_prefetch_miss_display = 8'h00;
reg         obj_prefetch_miss_line_seen = 1'b0;
reg  [7:0]  obj_urgent_wait_count = 8'h00;
reg  [7:0]  obj_urgent_wait_latched = 8'h00;
reg  [7:0]  obj_urgent_wait_peak = 8'h00;
reg  [7:0]  obj_urgent_wait_display = 8'h00;
reg         obj_urgent_wait_line_seen = 1'b0;
reg  [7:0]  obj_miss_cause_count [0:5];
reg  [7:0]  obj_miss_cause_latched [0:5];
reg  [7:0]  obj_miss_cause_peak [0:5];
reg  [7:0]  obj_miss_cause_display [0:5];
reg  [5:0]  obj_miss_cause_line_seen = 6'b000000;
reg  [5:0]  obj_miss_cause_count_event = 6'b000000;
reg         obj_prefetch_miss_count_event = 1'b0;
reg         obj_urgent_wait_count_event = 1'b0;
reg  [6:0]  pressure_hold_count = 7'd0;
reg  [7:0]  gp0_obj_live_total = 8'h00;
reg  [7:0]  gp1_obj_live_total = 8'h00;
reg  [7:0]  gp0_obj_visible_total = 8'h00;
reg  [7:0]  gp1_obj_visible_total = 8'h00;
reg  [7:0]  gp0_obj_visible_overflow_total = 8'h00;
reg  [7:0]  gp1_obj_visible_overflow_total = 8'h00;
reg  [7:0]  gp0_obj_live_latched = 8'h00;
reg  [7:0]  gp1_obj_live_latched = 8'h00;
reg  [7:0]  gp0_obj_visible_latched = 8'h00;
reg  [7:0]  gp1_obj_visible_latched = 8'h00;
reg  [7:0]  gp0_obj_cached_latched = 8'h00;
reg  [7:0]  gp1_obj_cached_latched = 8'h00;
reg  [7:0]  gp0_obj_visible_overflow_latched = 8'h00;
reg  [7:0]  gp1_obj_visible_overflow_latched = 8'h00;
reg  [7:0]  gp0_obj_live_peak = 8'h00;
reg  [7:0]  gp1_obj_live_peak = 8'h00;
reg  [7:0]  gp0_obj_visible_peak = 8'h00;
reg  [7:0]  gp1_obj_visible_peak = 8'h00;
reg  [7:0]  gp0_obj_cached_peak = 8'h00;
reg  [7:0]  gp1_obj_cached_peak = 8'h00;
reg  [7:0]  gp0_obj_visible_overflow_peak = 8'h00;
reg  [7:0]  gp1_obj_visible_overflow_peak = 8'h00;
reg  [7:0]  gp0_obj_live_display = 8'h00;
reg  [7:0]  gp1_obj_live_display = 8'h00;
reg  [7:0]  gp0_obj_visible_display = 8'h00;
reg  [7:0]  gp1_obj_visible_display = 8'h00;
reg  [7:0]  gp0_obj_cached_display = 8'h00;
reg  [7:0]  gp1_obj_cached_display = 8'h00;
reg  [7:0]  gp0_obj_visible_overflow_display = 8'h00;
reg  [7:0]  gp1_obj_visible_overflow_display = 8'h00;
integer pressure_i;
reg  [15:0] load_word0 = 16'h0000;
reg  [15:0] load_word1 = 16'h0000;
reg  [15:0] load_word2 = 16'h0000;
reg  [15:0] load_word3 = 16'h0000;
reg  [7:0] load_seen = 8'h00;
reg        last_ioctl_rom = 1'b0;
reg        ever_ioctl_rom = 1'b0;
reg        ever_ioctl_wr = 1'b0;
reg        ever_dwnld_busy = 1'b0;
reg        ever_prog_we = 1'b0;
reg        ever_prog_ack = 1'b0;
reg        ever_rom_probe_rd = 1'b0;
reg        ever_rom_probe_ok = 1'b0;
reg  [7:0] diag_ioctl_wr_count = 8'h00;
reg  [7:0] diag_last_ioctl_dout = 8'h00;
reg  [15:0] diag_last_ioctl_addr = 16'h0000;

wire        cpu_cen;
wire        cpu_cenb;
wire [23:1] cpu_addr;
wire [23:0] cpu_addr8 = {cpu_addr, 1'b0};
wire [15:0] cpu_dout;
reg  [15:0] cpu_din = 16'hffff;
wire [15:0] cpu_din_mux;
wire        cpu_rw;
wire        cpu_as_n;
wire        cpu_lds_n;
wire        cpu_uds_n;
wire        cpu_dtack_n;
wire        cpu_bg_n;
wire        cpu_fc0;
wire        cpu_fc1;
wire        cpu_fc2;
wire        rom_probe_match;
wire        cpu_vpa_n = ~&{cpu_fc0, cpu_fc1, cpu_fc2, ~cpu_as_n};
wire        cpu_iack = !cpu_as_n && cpu_fc0 && cpu_fc1 && cpu_fc2;
wire        cpu_bus_active = !cpu_as_n && (!cpu_uds_n || !cpu_lds_n);
wire        ss_handler_cs = ss_override && cpu_bus_active &&
                            (cpu_addr8[23:8] == 16'hff00);
wire        ss_reset_vector_cs = ss_override && cpu_bus_active &&
                                 (cpu_addr8 < 24'h000008);
wire        ss_irq_vector_cs = ss_override && cpu_bus_active &&
                               ((cpu_addr8 == 24'h00007c) ||
                                (cpu_addr8 == 24'h00007e));
wire        ss_special_cs = ss_handler_cs ||
                            ss_reset_vector_cs ||
                            ss_irq_vector_cs;
wire        cpu_reset_base = rst || dwnld_busy || ioctl_rom ||
                             !rom_probe_match || sound_diag_core_reset;
wire        cpu_program_space = cpu_fc1 && !cpu_fc0;
reg  [14:0] v25_stub_addr = 15'h0000;
reg         v25_stub_done = 1'b0;
wire        v25_preclear_we = !cpu_reset_base && !v25_stub_done;
wire        cpu_reset = cpu_reset_base || !v25_stub_done;
wire        cpu_core_reset = cpu_reset || ss_reset;
reg  [7:0]  v25_cen_accum = 8'd0;
reg         v25_cen = 1'b0;
wire [14:0] v25_shared_addr;
wire [7:0]  v25_shared_dout;
wire [7:0]  v25_shared_din;
wire        v25_shared_we;
wire [8:0]  v25_boot_shadow_addr;
wire [7:0]  v25_boot_shadow_data;
wire signed [15:0] sound_mono;
wire        sound_sample;
wire [17:0] oki_rom_addr;
wire [7:0]  oki_rom_dout;
wire        oki_rom_ok;
wire        oki_rom_rd;
wire [AW-1:0] oki_sdram_addr;
wire        sound_debug_fault;
wire        sound_debug_halted;
wire [19:0] sound_debug_pc;
wire        sound_debug_ym_write;
wire        sound_debug_ym_a0;
wire [7:0]  sound_debug_ym_data;
wire        sound_debug_oki_write;
wire [7:0]  sound_debug_oki_data;
wire        sound_debug_cdc_overrun;
wire        sound_v25_state_idle;
wire        sound_state_idle;
wire        sound_state_held;
wire        cpu_read = cpu_bus_active && cpu_rw;
wire        cpu_write = cpu_bus_active && !cpu_rw;
wire        cpu_rom_bus_cs = cpu_bus_active && !ss_special_cs &&
                             (cpu_addr8 < 24'h080000);
wire        cpu_rom_cs = cpu_read && (cpu_addr8 < 24'h080000);
wire        cpu_wram_cs = cpu_bus_active && (cpu_addr8 >= 24'h100000) && (cpu_addr8 < 24'h110000);
wire        cpu_in1_cs = cpu_read && (cpu_addr8 == 24'h200010);
wire        cpu_in2_cs = cpu_read && (cpu_addr8 == 24'h200014);
wire        cpu_sys_cs = cpu_read && (cpu_addr8 == 24'h200018);
wire        cpu_sound_reset_cs = cpu_bus_active && ((cpu_addr8 & 24'hfffffe) == 24'h20001c);
wire        cpu_shared_cs = cpu_bus_active && (cpu_addr8 >= 24'h210000) && (cpu_addr8 < 24'h220000);
wire        cpu_gp0_cs = cpu_bus_active && (cpu_addr8 >= 24'h300000) && (cpu_addr8 < 24'h300010);
wire        cpu_pal_cs = cpu_bus_active && (cpu_addr8 >= 24'h400000) && (cpu_addr8 < 24'h401000);
wire        cpu_gp1_cs = cpu_bus_active && (cpu_addr8 >= 24'h500000) && (cpu_addr8 < 24'h500010);
wire        cpu_vcnt_cs = cpu_read && ((cpu_addr8 & 24'hfffffe) == 24'h700000);
wire        cpu_io_cs = cpu_in1_cs || cpu_in2_cs || cpu_sys_cs || cpu_sound_reset_cs || cpu_vcnt_cs;
wire        cpu_gp_cs = cpu_gp0_cs || cpu_gp1_cs;
wire        cpu_gp_read = cpu_gp_cs && cpu_read;
wire        cpu_gp_write = cpu_gp_cs && cpu_write;
wire        cpu_gp_data_port = cpu_gp_cs && ((cpu_addr8[3:1] & 3'b110) == 3'b010);
reg         gp_bus_started = 1'b0;
reg         gp_bus_done = 1'b0;
wire        gp_start = cpu_gp_cs && !gp_bus_started && !gp_bus_done;
wire        gp0_start = gp_start && cpu_gp0_cs;
wire        gp1_start = gp_start && cpu_gp1_cs;
wire        gp0_busy;
wire        gp1_busy;
wire        gp0_done;
wire        gp1_done;
wire        gp0_irq_clear;
wire        gp1_irq_clear;
wire [15:0] gp0_dout;
wire [15:0] gp1_dout;
wire [12:0] gp0_dbg_ptr;
wire [12:0] gp1_dbg_ptr;
wire [15:0] gp0_dbg_last_addr;
wire [15:0] gp1_dbg_last_addr;
wire [15:0] gp0_dbg_last_din;
wire [15:0] gp1_dbg_last_din;
wire [15:0] gp0_dbg_last_dout;
wire [15:0] gp1_dbg_last_dout;
wire [15:0] gp0_dbg_regs_01;
wire [15:0] gp1_dbg_regs_01;
wire [15:0] gp0_dbg_regs_23;
wire [15:0] gp1_dbg_regs_23;
wire [15:0] gp0_scroll0;
wire [15:0] gp0_scroll1;
wire [15:0] gp0_scroll2;
wire [15:0] gp0_scroll3;
wire [15:0] gp0_scroll4;
wire [15:0] gp0_scroll5;
wire [15:0] gp0_scroll6;
wire [15:0] gp0_scroll7;
wire [15:0] gp1_scroll0;
wire [15:0] gp1_scroll1;
wire [15:0] gp1_scroll2;
wire [15:0] gp1_scroll3;
wire [15:0] gp1_scroll4;
wire [15:0] gp1_scroll5;
wire [15:0] gp1_scroll6;
wire [15:0] gp1_scroll7;
wire gp0_obj_cpu_write = gp0_start && cpu_gp_write && cpu_gp_data_port &&
                         (gp0_dbg_ptr >= 13'h1800) && (gp0_dbg_ptr < 13'h1c00);
wire gp1_obj_cpu_write = gp1_start && cpu_gp_write && cpu_gp_data_port &&
                         (gp1_dbg_ptr >= 13'h1800) && (gp1_dbg_ptr < 13'h1c00);
wire gp0_scroll_cpu_write = gp0_start && cpu_gp_write &&
                            (cpu_addr8[3:2] == 2'b11);
wire gp1_scroll_cpu_write = gp1_start && cpu_gp_write &&
                            (cpu_addr8[3:2] == 2'b11);
reg  [15:0] gp0_scroll0_px = 16'h0000;
reg  [15:0] gp0_scroll1_px = 16'h0000;
reg  [15:0] gp0_scroll2_px = 16'h0000;
reg  [15:0] gp0_scroll3_px = 16'h0000;
reg  [15:0] gp0_scroll4_px = 16'h0000;
reg  [15:0] gp0_scroll5_px = 16'h0000;
reg  [15:0] gp0_scroll6_px = 16'h0000;
reg  [15:0] gp0_scroll7_px = 16'h0000;
reg  [15:0] gp1_scroll0_px = 16'h0000;
reg  [15:0] gp1_scroll1_px = 16'h0000;
reg  [15:0] gp1_scroll2_px = 16'h0000;
reg  [15:0] gp1_scroll3_px = 16'h0000;
reg  [15:0] gp1_scroll4_px = 16'h0000;
reg  [15:0] gp1_scroll5_px = 16'h0000;
reg  [15:0] gp1_scroll6_px = 16'h0000;
reg  [15:0] gp1_scroll7_px = 16'h0000;
wire [15:0] gp0_scan_dout;
wire [15:0] gp1_scan_dout;
wire [15:0] gp0_obj_scan_dout;
wire [15:0] gp1_obj_scan_dout;
reg  [12:0] gp0_obj_scan_addr = 13'h1800;
reg  [12:0] gp1_obj_scan_addr = 13'h1800;
reg         obj_cache_scan_active = 1'b0;
wire        gp0_obj_buf_busy;
wire        gp1_obj_buf_busy;
wire        gp0_obj_forward_active;
wire        gp1_obj_forward_active;
wire        gp0_obj_forward_write;
wire        gp1_obj_forward_write;
wire        gp0_obj_forward_miss;
wire        gp1_obj_forward_miss;
wire        gp0_obj_live_candidate = gp_obj_entry_live_calc(
    gp0_obj_word0, gp0_obj_word1, gp0_obj_word2, gp0_obj_word3
);
wire        gp1_obj_live_candidate = gp_obj_entry_live_calc(
    gp1_obj_word0, gp1_obj_word1, gp1_obj_word2, gp1_obj_word3
);
wire [7:0]  gp0_obj_w_calc = gp_obj_size_calc(gp0_obj_word2[3:0]);
wire [7:0]  gp0_obj_h_calc = gp_obj_size_calc(gp0_obj_word3[3:0]);
wire [7:0]  gp1_obj_w_calc = gp_obj_size_calc(gp1_obj_word2[3:0]);
wire [7:0]  gp1_obj_h_calc = gp_obj_size_calc(gp1_obj_word3[3:0]);
wire [1:0]  gp0_obj_scroll_basis = status[28:27];
wire [15:0] gp0_obj_scroll_x_sel =
    (gp0_obj_scroll_basis == 2'd1) ? gp0_scroll0_px :
    (gp0_obj_scroll_basis == 2'd2) ? gp0_scroll2_px :
    (gp0_obj_scroll_basis == 2'd3) ? gp0_scroll4_px :
                                     gp0_scroll6_px;
wire [15:0] gp0_obj_scroll_y_sel =
    (gp0_obj_scroll_basis == 2'd1) ? gp0_scroll1_px :
    (gp0_obj_scroll_basis == 2'd2) ? gp0_scroll3_px :
    (gp0_obj_scroll_basis == 2'd3) ? gp0_scroll5_px :
                                     gp0_scroll7_px;
wire [15:0] gp0_obj_scroll_x = OBJ_SCROLL_BYPASS_TEST ? 16'h0000 : gp0_obj_scroll_x_sel;
wire [15:0] gp0_obj_scroll_y = OBJ_SCROLL_BYPASS_TEST ? 16'h0000 : gp0_obj_scroll_y_sel;
wire [15:0] gp1_obj_scroll_x = OBJ_SCROLL_BYPASS_TEST ? 16'h0000 : gp1_scroll6_px;
wire [15:0] gp1_obj_scroll_y = OBJ_SCROLL_BYPASS_TEST ? 16'h0000 : gp1_scroll7_px;
wire [8:0]  gp0_obj_base_x = gp0_obj_word0[14] ?
    gp_obj_rel_base_calc(gp0_obj_old_x, gp0_obj_word2) :
    gp_obj_abs_base_calc(gp0_obj_word2, 16'h0000);
wire [8:0]  gp0_obj_base_y = gp0_obj_word0[14] ?
    gp_obj_rel_base_calc(gp0_obj_old_y, gp0_obj_word3) :
    gp_obj_abs_base_calc(gp0_obj_word3, 16'h0000);
wire [8:0]  gp1_obj_base_x = gp1_obj_word0[14] ?
    gp_obj_rel_base_calc(gp1_obj_old_x, gp1_obj_word2) :
    gp_obj_abs_base_calc(gp1_obj_word2, 16'h0000);
wire [8:0]  gp1_obj_base_y = gp1_obj_word0[14] ?
    gp_obj_rel_base_calc(gp1_obj_old_y, gp1_obj_word3) :
    gp_obj_abs_base_calc(gp1_obj_word3, 16'h0000);
wire [8:0]  gp0_obj_draw_x = gp_obj_draw_x_from_raw_calc(
    gp0_obj_word0, gp0_obj_base_x, gp0_obj_scroll_x, gp0_obj_w_calc
);
wire [8:0]  gp0_obj_draw_y = gp_obj_draw_y_from_raw_calc(
    gp0_obj_word0, gp0_obj_base_y, gp0_obj_scroll_y, gp0_obj_h_calc
);
wire [8:0]  gp1_obj_draw_x = gp_obj_draw_x_from_raw_calc(
    gp1_obj_word0, gp1_obj_base_x, gp1_obj_scroll_x, gp1_obj_w_calc
);
wire [8:0]  gp1_obj_draw_y = gp_obj_draw_y_from_raw_calc(
    gp1_obj_word0, gp1_obj_base_y, gp1_obj_scroll_y, gp1_obj_h_calc
);
wire gp0_obj_stage_visible = gp_obj_rect_visible_calc(gp0_obj_stage_draw_x,
                                                      gp0_obj_stage_draw_y,
                                                      gp0_obj_stage_w,
                                                      gp0_obj_stage_h);
wire gp1_obj_stage_visible = gp_obj_rect_visible_calc(gp1_obj_stage_draw_x,
                                                      gp1_obj_stage_draw_y,
                                                      gp1_obj_stage_w,
                                                      gp1_obj_stage_h);
wire gp0_obj_stage_cacheable = gp0_obj_stage_live &&
                               (!OBJ_VISIBLE_CACHE_TEST || gp0_obj_stage_visible);
wire gp1_obj_stage_cacheable = gp1_obj_stage_live &&
                               (!OBJ_VISIBLE_CACHE_TEST || gp1_obj_stage_visible);
wire gp0_obj_stage_store = gp0_obj_stage_cacheable &&
                           (gp0_obj_count < OBJ_BOX_COUNT);
wire gp1_obj_stage_store = gp1_obj_stage_cacheable &&
                           (gp1_obj_count < OBJ_BOX_COUNT);
wire gp0_obj_stage_visible_overflow = gp0_obj_stage_live &&
                                      gp0_obj_stage_visible &&
                                      (gp0_obj_count >= OBJ_BOX_COUNT);
wire gp1_obj_stage_visible_overflow = gp1_obj_stage_live &&
                                      gp1_obj_stage_visible &&
                                      (gp1_obj_count >= OBJ_BOX_COUNT);
wire [7:0] gp0_obj_live_total_next = gp0_obj_stage_live ?
                                     sat8_inc(gp0_obj_live_total) :
                                     gp0_obj_live_total;
wire [7:0] gp1_obj_live_total_next = gp1_obj_stage_live ?
                                     sat8_inc(gp1_obj_live_total) :
                                     gp1_obj_live_total;
wire [7:0] gp0_obj_visible_total_next =
    (gp0_obj_stage_live && gp0_obj_stage_visible) ?
    sat8_inc(gp0_obj_visible_total) : gp0_obj_visible_total;
wire [7:0] gp1_obj_visible_total_next =
    (gp1_obj_stage_live && gp1_obj_stage_visible) ?
    sat8_inc(gp1_obj_visible_total) : gp1_obj_visible_total;
wire [7:0] gp0_obj_visible_overflow_total_next =
    gp0_obj_stage_visible_overflow ?
    sat8_inc(gp0_obj_visible_overflow_total) :
    gp0_obj_visible_overflow_total;
wire [7:0] gp1_obj_visible_overflow_total_next =
    gp1_obj_stage_visible_overflow ?
    sat8_inc(gp1_obj_visible_overflow_total) :
    gp1_obj_visible_overflow_total;
wire [7:0] gp0_obj_cached_next = {1'b0, gp0_obj_count} +
                                  (gp0_obj_stage_store ? 8'd1 : 8'd0);
wire [7:0] gp1_obj_cached_next = {1'b0, gp1_obj_count} +
                                  (gp1_obj_stage_store ? 8'd1 : 8'd0);
reg  [12:0] gp0_sample_addr = 13'h0000;
reg  [12:0] gp1_sample_addr = 13'h0000;
reg  [15:0] gp0_sample_word = 16'h0000;
reg  [15:0] gp1_sample_word = 16'h0000;
wire        cpu_gp_wait = cpu_gp_cs && !gp_bus_done;
wire        cpu_pal_write = cpu_pal_cs && cpu_write;
wire [1:0]  cpu_pal_we = {cpu_pal_write && !cpu_uds_n, cpu_pal_write && !cpu_lds_n};
wire        cpu_mapped_cs = cpu_rom_bus_cs || cpu_wram_cs || cpu_shared_cs || cpu_gp_cs ||
                            cpu_pal_cs || cpu_io_cs || ss_special_cs;
wire        cpu_unmap_cs = cpu_bus_active && !cpu_mapped_cs;
wire        cpu_bus_cs = cpu_bus_active;
wire        cpu_bus_busy = (cpu_rom_cs && !rom_slot_ok) || cpu_gp_wait;
wire        cpu_ram_write = cpu_wram_cs && cpu_write;
wire        cpu_shared_write = cpu_shared_cs && cpu_write;
wire        cpu_shared_read = cpu_shared_cs && cpu_read;
wire [15:0] cpu_wram_q;
wire [15:0] cpu_pal_q;
wire [7:0]  cpu_shared_q;
wire [7:0]  cpu_shared_din = cpu_dout[7:0];
wire [15:0] cpu_shared_data = {8'h00, cpu_shared_q};
wire [1:0]  cpu_wram_we = {cpu_ram_write && !cpu_uds_n, cpu_ram_write && !cpu_lds_n};
wire        cpu_shared_we = cpu_shared_write && (!cpu_uds_n || !cpu_lds_n);
wire [15:0] cpu_in1_data;
wire [15:0] cpu_in2_data;
wire [15:0] cpu_sys_data;

batsugun_inputs u_inputs (
    .joy1_n     ( joystick1[6:0] ),
    .joy2_n     ( joystick2[6:0] ),
    .start_n    ( cab_1p         ),
    .coin_n     ( coin           ),
    .service_n  ( service        ),
    .tilt_n     ( tilt           ),
    .test_n     ( dip_test       ),
    .in1        ( cpu_in1_data   ),
    .in2        ( cpu_in2_data   ),
    .sys        ( cpu_sys_data   )
);
wire [9:0]  vdp_status_sum = {1'b0, vcnt} + 10'd15;
wire [9:0]  vdp_status_v = (vdp_status_sum >= 10'd262) ? vdp_status_sum - 10'd262 : vdp_status_sum;
wire        vdp_hsync_n = ~((hcnt > 10'd325) && (hcnt < 10'd380));
wire        vdp_vsync_n = ~((vcnt >= 9'd232) && (vcnt <= 9'd245));
wire        vdp_fblank_n = vdp_hsync_n && vdp_vsync_n;
wire [15:0] vdp_count_flags = 16'hff00 &
                              (vdp_hsync_n ? 16'hffff : 16'h7fff) &
                              (vdp_vsync_n ? 16'hffff : 16'hbfff) &
                              (vdp_fblank_n ? 16'hffff : 16'hfeff);
wire [15:0] cpu_vcnt_data = vdp_count_flags |
                            ((vdp_status_v < 10'd256) ? {8'h00, vdp_status_v[7:0]} : 16'h00ff);
wire        cpu_gp_status = vdp_status_v >= 10'd245;
wire [15:0] cpu_gp_data = cpu_gp1_cs ? gp1_dout : gp0_dout;
reg        fault_snapshot_seen = 1'b0;
reg [15:0] fault_gp_addr = 16'h0000;
reg [15:0] fault_gp_data = 16'h0000;
reg [15:0] fault_gp0_latch = 16'h0000;
reg [15:0] fault_gp1_latch = 16'h0000;
reg [15:0] fault_bus_counts = 16'h0000;
reg        gp_data_test_seen = 1'b0;
assign cpu_din_mux = ss_handler_cs ? ss_irq_handler_word(cpu_addr8[4:1]) :
                     ss_reset_vector_cs ? ss_reset_vector[cpu_addr8[2:1]] :
                     ss_irq_vector_cs ? (cpu_addr8[1] ? 16'h0000 : 16'h00ff) :
                     cpu_rom_cs    ? rom_slot_dout :
                     cpu_wram_cs   ? cpu_wram_q :
                     cpu_shared_cs ? cpu_shared_data :
                     cpu_gp_cs     ? cpu_gp_data :
                     cpu_pal_cs    ? cpu_pal_q :
                     cpu_in1_cs    ? cpu_in1_data :
                     cpu_in2_cs    ? cpu_in2_data :
                     cpu_sys_cs    ? cpu_sys_data :
                     cpu_vcnt_cs   ? cpu_vcnt_data :
                     16'hffff;

// DTACK already gives every read at least one CPU wait state. Register the
// completed mux in the master-clock domain so fx68k never samples its long
// VDP/RAM/IO combinational path directly.
always @(posedge clk) begin
    if (cpu_reset)
        cpu_din <= 16'hffff;
    else if (cpu_read)
        cpu_din <= cpu_din_mux;
end
reg        cpu_ack_seen = 1'b0;
wire       cpu_ack_now = cpu_bus_active && !cpu_dtack_n && !cpu_ack_seen;
wire       cpu_rom_ack = cpu_ack_now && cpu_rom_bus_cs && cpu_read && cpu_program_space;
wire       cpu_fatal_fetch = cpu_rom_ack && (cpu_addr8 >= 24'h03bf00) && (cpu_addr8 < 24'h03bf10);
wire       cpu_nonfatal_rom_fetch = cpu_rom_ack && !cpu_fatal_fetch;
`ifdef BATSUGUN_HW_DEBUG
// A sequential ROM fetch can be speculative on the 68000. Use bus cycles that
// prove which side of the service-mode branch actually committed instead.
wire       cpu_boot_branch_fetch = cpu_rom_ack &&
                                   (cpu_addr8 == 24'h03bdca) &&
                                   (cpu_din == 16'h6744);
wire       cpu_service_commit = cpu_ack_now && cpu_write && cpu_wram_cs &&
                                (cpu_addr8 == 24'h1011fa);
wire       cpu_normal_boot_fetch = cpu_rom_ack &&
                                   (cpu_addr8 == 24'h000882);
`endif
reg [15:0] cpu_bus_count = 16'h0000;
reg [7:0]  cpu_rom_count = 8'h00;
reg [7:0]  cpu_wram_count = 8'h00;
reg [7:0]  cpu_shared_count = 8'h00;
reg [7:0]  cpu_gp_count = 8'h00;
reg [7:0]  cpu_pal_count = 8'h00;
reg [7:0]  cpu_io_count = 8'h00;
reg [7:0]  cpu_unmap_count = 8'h00;
reg [23:0] cpu_last_addr = 24'h000000;
reg [15:0] cpu_last_dout = 16'h0000;
reg [15:0] cpu_last_din = 16'h0000;
reg [15:0] cpu_display_word;
reg        ever_cpu_shared = 1'b0;
reg        ever_cpu_gp = 1'b0;
reg        ever_cpu_pal = 1'b0;
reg        ever_cpu_io = 1'b0;
reg        ever_cpu_unmap = 1'b0;
reg [7:0]  cpu_shared_read_count = 8'h00;
reg [7:0]  cpu_shared_write_count = 8'h00;
reg [15:0] cpu_last_shared_addr = 16'h0000;
reg [7:0]  cpu_last_shared_dout = 8'h00;
reg [7:0]  cpu_last_shared_din = 8'h00;
reg [15:0] cpu_last_gp_addr = 16'h0000;
reg [15:0] cpu_last_gp_dout = 16'h0000;
reg [15:0] cpu_last_pal_dout = 16'h0000;
reg        cpu_irq4 = 1'b0;
reg        ever_cpu_irq4 = 1'b0;
reg        ever_cpu_iack = 1'b0;
reg        ever_sound_reset = 1'b0;
reg        ever_sound_release = 1'b0;
reg        v25_sound_released = 1'b0;

wire [63:0] ss_current_rom_signature = {
    rom_probe_word0,
    rom_probe_word1,
    rom_probe_word2,
    rom_probe_word3
};
wire ss_restore_compatible =
    ss_format_valid &&
    (ss_restore_rom_signature == ss_current_rom_signature);
wire ss_video_release_point =
    ss_video_frame_seen &&
    (clkdiv == 4'd0) &&
    (hcnt == 10'd0) &&
    (vcnt == V_END);
wire ss_safe_point =
    !cpu_reset_base &&
    v25_stub_done &&
    !render_lvbl &&
    (vcnt == V_END) &&
    !cpu_bus_active &&
    sound_state_idle &&
    !gp0_busy &&
    !gp1_busy &&
    !gp0_obj_buf_busy &&
    !gp1_obj_buf_busy;

reg  [63:0] ss_global_data_out = 64'd0;
reg         ss_global_ack = 1'b0;
wire [63:0] ss_wram_data_out;
wire [63:0] ss_shared_data_out;
wire [63:0] ss_palette_data_out;
wire [63:0] ss_gp0_data_out;
wire [63:0] ss_gp1_data_out;
wire [63:0] ss_v25_data_out;
wire        ss_wram_ack;
wire        ss_shared_ack;
wire        ss_palette_ack;
wire        ss_gp0_ack;
wire        ss_gp1_ack;
wire        ss_v25_ack;

assign ss_ack = ss_global_ack ||
                ss_wram_ack ||
                ss_shared_ack ||
                ss_palette_ack ||
                ss_gp0_ack ||
                ss_gp1_ack ||
                ss_v25_ack;
assign ss_data_out = ss_global_ack ? ss_global_data_out :
                     ss_wram_ack ? ss_wram_data_out :
                     ss_shared_ack ? ss_shared_data_out :
                     ss_palette_ack ? ss_palette_data_out :
                     ss_gp0_ack ? ss_gp0_data_out :
                     ss_gp1_ack ? ss_gp1_data_out :
                     ss_v25_ack ? ss_v25_data_out :
                     64'd0;

// Main-to-V25 commands use 7801 for the argument and 7800 for the command.
// Captured game traffic identifies argument 00 as BGM, while 00/01 stops it.
// Preserve only that durable music intent; replaying voice or SFX on load
// would create sounds that did not exist at the restored gameplay instant.
always @(posedge clk) begin
    if (rst || dwnld_busy || ioctl_rom) begin
        sound_bgm_pending_argument <= 8'h00;
        sound_bgm_command <= 8'h00;
        sound_bgm_argument <= 8'h00;
        sound_bgm_valid <= 1'b0;
    end else if (ss_restore_commit) begin
        sound_bgm_pending_argument <= ss_restore_bgm_argument;
        sound_bgm_command <= ss_restore_bgm_command;
        sound_bgm_argument <= ss_restore_bgm_argument;
        sound_bgm_valid <= ss_restore_bgm_valid;
    end else if (cpu_ack_now && cpu_shared_we) begin
        if (cpu_addr8[15:1] == 15'h7801) begin
            sound_bgm_pending_argument <= cpu_shared_din;
        end else if (cpu_addr8[15:1] == 15'h7800 &&
                     cpu_shared_din != 8'hff) begin
            if (cpu_shared_din == 8'h00 &&
                sound_bgm_pending_argument == 8'h01) begin
                sound_bgm_valid <= 1'b0;
            end else if (sound_bgm_pending_argument == 8'h00) begin
                sound_bgm_command <= cpu_shared_din;
                sound_bgm_argument <= sound_bgm_pending_argument;
                sound_bgm_valid <= 1'b1;
            end
        end
    end
end

// Chunk 1: 68000 supervisor stack pointer, interrupt/sound release flags,
// current BGM intent, and the loaded program-ROM signature. The BGM fields use
// formerly-zero bits, so old states remain structurally compatible and load
// with BGM replay disabled. RAM and device chunks begin at index 2.
always @(posedge clk) begin
    ss_global_ack <= 1'b0;

    if (rst) begin
        ss_global_data_out <= 64'd0;
        ss_restore_ssp <= 32'd0;
        ss_restore_irq4 <= 1'b0;
        ss_restore_sound_released <= 1'b0;
        ss_restore_bgm_command <= 8'h00;
        ss_restore_bgm_argument <= 8'h00;
        ss_restore_bgm_valid <= 1'b0;
        ss_restore_rom_signature <= 64'd0;
    end else if (ss_do_restore) begin
        ss_restore_ssp <= 32'd0;
        ss_restore_irq4 <= 1'b0;
        ss_restore_sound_released <= 1'b0;
        ss_restore_bgm_command <= 8'h00;
        ss_restore_bgm_argument <= 8'h00;
        ss_restore_bgm_valid <= 1'b0;
        ss_restore_rom_signature <= 64'd0;
    end else if (ss_select == SSIDX_GLOBAL) begin
        if (ss_query) begin
            ss_global_data_out <= {SSIDX_GLOBAL, 22'd0, 2'd3, 32'd2};
            ss_global_ack <= 1'b1;
        end else if (ss_read) begin
            if (ss_addr[0])
                ss_global_data_out <= ss_current_rom_signature;
            else
                ss_global_data_out <= {
                    13'd0,
                    sound_bgm_valid,
                    sound_bgm_argument,
                    sound_bgm_command,
                    v25_sound_released,
                    cpu_irq4,
                    ss_saved_ssp
                };
            ss_global_ack <= 1'b1;
        end else if (ss_write) begin
            if (ss_addr[0]) begin
                ss_restore_rom_signature <= ss_data;
            end else begin
                ss_restore_ssp <= ss_data[31:0];
                ss_restore_irq4 <= ss_data[32];
                ss_restore_sound_released <= ss_data[33];
                ss_restore_bgm_command <= ss_data[41:34];
                ss_restore_bgm_argument <= ss_data[49:42];
                ss_restore_bgm_valid <= ss_data[50];
            end
            ss_global_ack <= 1'b1;
        end
    end
end

// Quiesce at the first idle bus slot in vertical blank. IRQ7 enters a tiny
// private handler that stacks every architectural 68000 register into work
// RAM, exposes SSP at ff0000, and later restores the same frame with RTE.
always @(posedge clk) begin
    if (rst || dwnld_busy || ioctl_rom) begin
        ss_state <= SS_IDLE;
        ss_write_start <= 1'b0;
        ss_read_start <= 1'b0;
        ss_reset_counter <= 8'd0;
        ss_saved_ssp <= 32'd0;
        ss_restore_commit <= 1'b0;
        ss_video_frame_seen <= 1'b0;
        ss_reset_vector[0] <= 16'd0;
        ss_reset_vector[1] <= 16'd0;
        ss_reset_vector[2] <= 16'd0;
        ss_reset_vector[3] <= 16'd0;
    end else begin
        ss_restore_commit <= 1'b0;
        if (ss_active && debug_frame_tick)
            ss_video_frame_seen <= 1'b1;

        case (ss_state)
            SS_IDLE: begin
                ss_write_start <= 1'b0;
                ss_read_start <= 1'b0;
                if (ss_do_save) begin
                    ss_state <= SS_SAVE_WAIT_SAFE;
                end else if (ss_do_restore) begin
                    ss_state <= SS_RESTORE_WAIT_SAFE;
                end
            end

            SS_SAVE_WAIT_SAFE: begin
                if (ss_safe_point)
                    ss_state <= SS_SAVE_WAIT_IRQ;
            end

            SS_SAVE_WAIT_IRQ: begin
                if (cpu_iack && (cpu_addr[3:1] == 3'b111) &&
                    !cpu_lds_n)
                    ss_state <= SS_SAVE_WAIT_SSP;
            end

            SS_SAVE_WAIT_SSP: begin
                if (cpu_ack_now && !cpu_rw &&
                    (cpu_addr8 == 24'hff0000))
                    ss_saved_ssp[31:16] <= cpu_dout;

                if (cpu_ack_now && !cpu_rw &&
                    (cpu_addr8 == 24'hff0002)) begin
                    ss_saved_ssp[15:0] <= cpu_dout;
                    ss_state <= SS_SAVE_WAIT_HOLD;
                end
            end

            SS_SAVE_WAIT_HOLD: begin
                if (sound_state_held) begin
                    ss_write_start <= 1'b1;
                    ss_state <= SS_SAVE_WAIT_STREAM;
                end
            end

            SS_SAVE_WAIT_STREAM: begin
                if (ss_busy && ss_write_start) begin
                    ss_write_start <= 1'b0;
                end else if (!ss_busy && !ss_write_start) begin
                    ss_video_frame_seen <= 1'b0;
                    ss_state <= SS_SAVE_WAIT_EXIT;
                end
            end

            SS_SAVE_WAIT_EXIT: begin
                if (cpu_ack_now && cpu_rw &&
                    ({cpu_fc2, cpu_fc1, cpu_fc0} == 3'b110) &&
                    !ss_handler_cs)
                    ss_state <= SS_SAVE_WAIT_VIDEO;
            end

            SS_SAVE_WAIT_VIDEO: begin
                if (ss_video_release_point)
                    ss_state <= SS_IDLE;
            end

            SS_RESTORE_WAIT_SAFE: begin
                if (ss_safe_point)
                    ss_state <= SS_RESTORE_WAIT_HOLD;
            end

            SS_RESTORE_WAIT_HOLD: begin
                if (sound_state_held) begin
                    ss_read_start <= 1'b1;
                    ss_state <= SS_RESTORE_WAIT_STREAM;
                end
            end

            SS_RESTORE_WAIT_STREAM: begin
                if (ss_busy && ss_read_start) begin
                    ss_read_start <= 1'b0;
                end else if (!ss_busy && !ss_read_start) begin
                    if (ss_restore_compatible) begin
                        ss_reset_vector[0] <= ss_restore_ssp[31:16];
                        ss_reset_vector[1] <= ss_restore_ssp[15:0];
                        ss_reset_vector[2] <= 16'h00ff;
                        ss_reset_vector[3] <= 16'h0008;
                        ss_reset_counter <= 8'd0;
                        ss_restore_commit <= 1'b1;
                        ss_video_frame_seen <= 1'b0;
                        ss_state <= SS_RESTORE_HOLD_RESET;
                    end else begin
                        ss_video_frame_seen <= 1'b0;
                        ss_state <= SS_RESTORE_WAIT_VIDEO;
                    end
                end
            end

            SS_RESTORE_HOLD_RESET: begin
                ss_reset_counter <= ss_reset_counter + 8'd1;
                if (&ss_reset_counter)
                    ss_state <= SS_RESTORE_WAIT_RESET;
            end

            SS_RESTORE_WAIT_RESET: begin
                if (cpu_ack_now && cpu_rw &&
                    ({cpu_fc2, cpu_fc1, cpu_fc0} == 3'b110) &&
                    !ss_handler_cs && !ss_reset_vector_cs)
                    ss_state <= SS_RESTORE_WAIT_VIDEO;
            end

            SS_RESTORE_WAIT_VIDEO: begin
                if (ss_video_release_point && sound_state_held)
                    ss_state <= SS_IDLE;
            end

            default: begin
                ss_state <= SS_IDLE;
                ss_write_start <= 1'b0;
                ss_read_start <= 1'b0;
            end
        endcase
    end
end

`ifdef BATSUGUN_SS_PROBE
reg [3:0]  ss_hw_prev_state = SS_IDLE;
reg [31:0] ss_hw_state_age = 32'd0;

always @(posedge clk) begin
    if (rst || dwnld_busy || ioctl_rom) begin
        ss_hw_prev_state <= SS_IDLE;
        ss_hw_state_age <= 32'd0;
    end else begin
        ss_hw_prev_state <= ss_state;
        if (ss_state == SS_IDLE || ss_state != ss_hw_prev_state)
            ss_hw_state_age <= 32'd0;
        else if (ss_hw_state_age != 32'hffffffff)
            ss_hw_state_age <= ss_hw_state_age + 32'd1;
    end
end

wire [127:0] ss_hw_probe = {
    16'h5353,
    ss_state,
    ss_active,
    ss_do_save,
    ss_do_restore,
    ss_write_start,
    ss_read_start,
    ss_busy,
    ss_safe_point,
    ss_video_release_point,
    cpu_reset_base,
    v25_stub_done,
    render_lvbl,
    cpu_bus_active,
    sound_state_idle,
    gp0_busy,
    gp1_busy,
    sound_state_held,
    gp0_obj_buf_busy,
    gp1_obj_buf_busy,
    cpu_iack,
    cpu_ack_now,
    vcnt,
    hcnt,
    clkdiv,
    cpu_addr8,
    cpu_fc2,
    cpu_fc1,
    cpu_fc0,
    cpu_rw,
    !cpu_lds_n,
    !cpu_uds_n,
    !cpu_dtack_n,
    ss_override,
    ss_hw_state_age
};
wire [159:0] ss_hw_probe_detail = {
    16'h5344,
    ss_format_valid,
    ss_restore_compatible,
    ss_restore_commit,
    rom_probe_done,
    ss_read_start,
    ss_write_start,
    ss_busy,
    ss_safe_point,
    sound_state_held,
    ss_state == SS_IDLE,
    ss_video_release_point,
    cpu_reset_base,
    v25_stub_done,
    ss_video_frame_seen,
    ss_reset,
    ss_override,
    ss_restore_rom_signature,
    ss_current_rom_signature
};
wire ss_hw_probe_source;
wire ss_hw_probe_detail_source;

altsource_probe #(
    .sld_auto_instance_index ("NO"),
    .sld_instance_index      (0),
    .instance_id             ("BSS"),
    .probe_width             (128),
    .source_width            (1),
    .source_initial_value    ("0"),
    .enable_metastability    ("NO")
) u_ss_hw_probe (
    .probe  (ss_hw_probe),
    .source (ss_hw_probe_source)
);

altsource_probe #(
    .sld_auto_instance_index ("NO"),
    .sld_instance_index      (1),
    .instance_id             ("SSD"),
    .probe_width             (160),
    .source_width            (1),
    .source_initial_value    ("0"),
    .enable_metastability    ("NO")
) u_ss_hw_probe_detail (
    .probe  (ss_hw_probe_detail),
    .source (ss_hw_probe_detail_source)
);
`endif

`ifdef BATSUGUN_HW_DEBUG
reg [23:0] sound_diag_frame_count = 24'h000000;
reg [15:0] sound_diag_main_cmd_count = 16'h0000;
reg [15:0] sound_diag_v25_ack_count = 16'h0000;
reg [15:0] sound_diag_ym_count = 16'h0000;
reg [15:0] sound_diag_oki_count = 16'h0000;
reg [7:0]  sound_diag_last_main_cmd = 8'h00;
reg [7:0]  sound_diag_last_main_arg = 8'h00;
reg [14:0] sound_diag_last_main_addr = 15'h0000;
reg [7:0]  sound_diag_last_main_data = 8'h00;
reg [14:0] sound_diag_last_v25_addr = 15'h0000;
reg [7:0]  sound_diag_last_v25_data = 8'h00;
reg [7:0]  sound_diag_last_ym_data = 8'h00;
reg [7:0]  sound_diag_last_oki_data = 8'h00;
reg [7:0]  sound_diag_sys_read_count = 8'h00;
reg [7:0]  sound_diag_last_sys_data = 8'h00;
reg        sound_diag_service_commit_seen = 1'b0;
reg        sound_diag_normal_boot_seen = 1'b0;
reg        sound_diag_boot_branch_armed = 1'b0;
reg [19:0] sound_diag_fetch_0 = 20'h00000;
reg [19:0] sound_diag_fetch_1 = 20'h00000;
reg [19:0] sound_diag_fetch_2 = 20'h00000;
reg [19:0] sound_diag_fetch_3 = 20'h00000;
reg [15:0] sound_diag_fetch_data_3bdc2 = 16'h0000;
reg [15:0] sound_diag_fetch_data_3bdc4 = 16'h0000;
reg [15:0] sound_diag_fetch_data_3bdc6 = 16'h0000;
reg [15:0] sound_diag_fetch_data_3bdc8 = 16'h0000;
reg [15:0] sound_diag_fetch_data_3bdca = 16'h0000;
reg [15:0] sound_diag_last_sys_word = 16'h0000;
reg [4:0]  sound_diag_fetch_data_seen = 5'b00000;
reg [15:0] sound_diag_dip_a_word = 16'h0000;
reg [15:0] sound_diag_dip_b_word = 16'h0000;
reg [15:0] sound_diag_region_word = 16'h0000;
reg [2:0]  sound_diag_dip_read_seen = 3'b000;
reg [15:0] sound_diag_v25_dip_write_count = 16'h0000;
reg [7:0]  sound_diag_v25_dip_last_data = 8'h00;
reg        sound_diag_v25_dip_bad_seen = 1'b0;
reg [7:0]  sound_diag_v25_dip_bad_data = 8'h00;
reg [19:0] sound_diag_v25_dip_bad_pc = 20'h00000;
reg [15:0] sound_diag_main_dip_write_count = 16'h0000;
reg [15:0] sound_diag_main_dip_write_data = 16'h0000;
reg        sound_diag_fault_sticky = 1'b0;
reg        sound_diag_halted_sticky = 1'b0;
reg        sound_diag_cdc_sticky = 1'b0;
`endif
reg        fatal_fetch_seen = 1'b0;
reg [23:0] rom_fetch_0 = 24'h000000;
reg [23:0] rom_fetch_1 = 24'h000000;
reg [23:0] rom_fetch_2 = 24'h000000;
reg [23:0] rom_fetch_3 = 24'h000000;
reg [7:0]  cpu_last_sound_reset = 8'h00;
wire       cpu_irq4_start = (clkdiv == 4'd13) && (hcnt == 10'd0) && (vcnt == 9'he6);
wire       cpu_sound_reset_write = cpu_ack_now && cpu_sound_reset_cs && cpu_write;
wire       cpu_sound_release_write = cpu_sound_reset_write && cpu_dout[5];
wire [15:0] cpu_irq_sound_flags = {
    cpu_irq4,
    ever_cpu_irq4,
    cpu_iack,
    ever_cpu_iack,
    !cpu_vpa_n,
    ever_sound_reset,
    ever_sound_release,
    fatal_fetch_seen,
    cpu_last_sound_reset
};
wire [15:0] cpu_diag_flags = {
    cpu_cen,
    cpu_cenb,
    !cpu_reset,
    cpu_bus_active,
    cpu_as_n,
    cpu_rw,
    cpu_uds_n,
    cpu_lds_n,
    !cpu_dtack_n,
    cpu_rom_bus_cs,
    cpu_wram_cs,
    cpu_shared_cs,
    cpu_gp_cs,
    cpu_pal_cs,
    cpu_io_cs,
    cpu_unmap_cs
};
wire [15:0] cpu_video_flags = {
    !cpu_reset,
    cpu_bus_active,
    cpu_ack_now,
    cpu_rom_bus_cs,
    cpu_wram_cs,
    cpu_shared_cs,
    cpu_gp0_cs,
    cpu_gp1_cs,
    cpu_pal_cs,
    cpu_io_cs,
    cpu_unmap_cs,
    ever_cpu_shared,
    ever_cpu_gp,
    ever_cpu_pal,
    ever_cpu_io,
    ever_cpu_unmap
};
wire load_done = &load_seen;
// Keep the ROM-load guard, but accept both official program revisions. The
// first three reset-vector words are shared; Special Version changes word 3.
wire load_program_signature =
    (load_word0 == 16'h0011) &&
    (load_word1 == 16'h0000) &&
    (load_word2 == 16'h0003) &&
    ((load_word3 == 16'hbc60) || (load_word3 == 16'hc460));
wire load_match = (load_seen == 8'hff) &&
                  load_program_signature;
wire rom_probe_done = &rom_probe_seen;
wire rom_probe_program_signature =
    (rom_probe_word0 == 16'h0011) &&
    (rom_probe_word1 == 16'h0000) &&
    (rom_probe_word2 == 16'h0003) &&
    ((rom_probe_word3 == 16'hbc60) || (rom_probe_word3 == 16'hc460));
assign rom_probe_match = (rom_probe_seen == 8'hff) &&
                         rom_probe_program_signature;
wire rom_probe_area = render_lhbl && render_lvbl &&
                      (hcnt >= 10'd32) && (hcnt < 10'd288) &&
                      (vcnt >= 9'd32) && (vcnt < 9'd160);
wire rom_status_area = render_lhbl && render_lvbl &&
                       (hcnt >= 10'd32) && (hcnt < 10'd288) &&
                       (vcnt >= 9'd176) && (vcnt < 9'd192);
wire [9:0] rom_probe_x = hcnt - 10'd32;
wire [8:0] rom_probe_y = vcnt - 9'd32;
wire [2:0] rom_probe_row = rom_probe_y[6:4];
wire [7:0] debug_hex_x = rom_probe_x[7:0] - 8'd16;
wire [1:0] debug_hex_digit = debug_hex_x[6:5];
wire [3:0] debug_hex_nibble = (debug_hex_digit == 2'd0) ? rom_probe_display_word[15:12] :
                               (debug_hex_digit == 2'd1) ? rom_probe_display_word[11:8] :
                               (debug_hex_digit == 2'd2) ? rom_probe_display_word[7:4] :
                                                           rom_probe_display_word[3:0];
wire [2:0] debug_hex_font_x = debug_hex_x[4:2];
wire [2:0] debug_hex_font_y = rom_probe_y[3:1];
wire [3:0] debug_hex_font_row = hex_glyph(debug_hex_nibble, debug_hex_font_y);
wire debug_hex_area = rom_probe_area && (rom_probe_x >= 10'd16) && (rom_probe_x < 10'd144);
wire [3:0] debug_view_sel = status[21:18];
localparam ENABLE_OBJ_LINE_DEBUG = 1'b0;
localparam ENABLE_OBJ_SPRITE_COMPOSITE = 1'b1;
localparam ENABLE_OBJ_SILHOUETTE_COMPOSITE = 1'b0;
localparam ENABLE_OBJ_CACHE_DEBUG = 1'b0;
localparam NO_OBJ_COMPOSITE_DIAG = 1'b0;
wire debug_object_box_en = (debug_view_sel == 4'd2) || (debug_view_sel == 4'd11) ||
                           (debug_view_sel == 4'd12);
wire debug_object_fill_en = (debug_view_sel == 4'd11) || (debug_view_sel == 4'd12);
wire debug_object_line_en = ENABLE_OBJ_LINE_DEBUG && (debug_view_sel == 4'd12);
wire debug_object_cache_en = ENABLE_OBJ_CACHE_DEBUG && debug_object_fill_en &&
                             !debug_object_line_en;
wire debug_object_overlay_en = debug_object_box_en || debug_object_fill_en;
wire debug_edge_src_en = debug_view_sel == 4'd14;
wire debug_overlay_en = |debug_view_sel;
wire debug_text_overlay_en = debug_overlay_en && !debug_object_overlay_en &&
                             !debug_edge_src_en;
wire debug_hex_bit = debug_hex_area && (debug_hex_font_y < 3'd7) &&
                     (debug_hex_font_x < 3'd4) &&
                     debug_hex_font_row[3'd3 - debug_hex_font_x[1:0]];
wire debug_hex_row_line = rom_probe_area && (rom_probe_y[3:0] == 4'd0);
wire gp_tile_area = render_lhbl && render_lvbl;
wire gp_render_sel = status[22];
wire debug_pressure_en = debug_view_sel == 4'd13;
wire debug_pressure_area = debug_pressure_en && gp_tile_area &&
                           (hcnt < 10'd128) && (vcnt < 9'd128);
wire [2:0] debug_pressure_row = vcnt[6:4];
wire [7:0] debug_pressure_x = hcnt[7:0];
wire [1:0] debug_pressure_digit = debug_pressure_x[6:5];
wire [2:0] debug_pressure_font_x = debug_pressure_x[4:2];
wire [2:0] debug_pressure_font_y = vcnt[3:1];
reg  [15:0] debug_pressure_word;
always @* begin
    if (gp_render_sel) begin
        case (debug_pressure_row)
            3'd0: debug_pressure_word = {8'ha0, obj_miss_cause_display[0]};
            3'd1: debug_pressure_word = {8'ha1, obj_miss_cause_display[1]};
            3'd2: debug_pressure_word = {8'hb0, obj_miss_cause_display[2]};
            3'd3: debug_pressure_word = {8'hb1, obj_miss_cause_display[3]};
            3'd4: debug_pressure_word = {8'hc0, obj_miss_cause_display[4]};
            3'd5: debug_pressure_word = {8'hc1, obj_miss_cause_display[5]};
            3'd6: debug_pressure_word = {8'he0, gp0_obj_visible_overflow_display};
            default: debug_pressure_word = {8'he1, gp1_obj_visible_overflow_display};
        endcase
    end else begin
        case (debug_pressure_row)
            3'd0: debug_pressure_word = {8'h00, comp_miss_display[0]};
            3'd1: debug_pressure_word = {8'h01, comp_miss_display[1]};
            3'd2: debug_pressure_word = {8'h02, comp_miss_display[2]};
            3'd3: debug_pressure_word = {8'h03, comp_miss_display[3]};
            3'd4: debug_pressure_word = {8'h04, comp_miss_display[4]};
            3'd5: debug_pressure_word = {8'h05, comp_miss_display[5]};
            3'd6: debug_pressure_word = {8'h0e, obj_prefetch_miss_display};
            default: debug_pressure_word = {8'h0f, obj_urgent_wait_display};
        endcase
    end
end
wire [3:0] debug_pressure_nibble =
    (debug_pressure_digit == 2'd0) ? debug_pressure_word[15:12] :
    (debug_pressure_digit == 2'd1) ? debug_pressure_word[11:8] :
    (debug_pressure_digit == 2'd2) ? debug_pressure_word[7:4] :
                                     debug_pressure_word[3:0];
wire [3:0] debug_pressure_font_row =
    hex_glyph(debug_pressure_nibble, debug_pressure_font_y);
wire debug_pressure_hex_bit = debug_pressure_area &&
                              (debug_pressure_font_y < 3'd7) &&
                              (debug_pressure_font_x < 3'd4) &&
                              debug_pressure_font_row[3'd3 - debug_pressure_font_x[1:0]];
wire [1:0] gp_render_layer_raw = status[24:23];
wire [1:0] gp_gfx_decode_mode = status[26:25];
wire [1:0] gp_gfx_map_mode = status[28:27];
wire [1:0] gp_tile_word_mode = status[30:29];
wire gp_tile_hex_mode = status[31];
wire [1:0] gp_live_gfx_decode_mode = GP_LIVE_GFX_DECODE_MODE;
wire [1:0] gp_live_gfx_map_mode = GP_LIVE_GFX_MAP_MODE;
wire [15:0] gfx_probe_slot_word = gfx_probe_slot_dout;
wire [1:0] gp_live_tile_word_mode = GP_LIVE_TILE_WORD_MODE;
localparam GP_LIVE_PLANE_SWAP = 1'b0;
wire obj_linebuffer_render_en = ENABLE_OBJ_LINEBUFFER_COMPOSITE &&
                                ENABLE_OBJ_SPRITE_COMPOSITE &&
                                !NO_OBJ_COMPOSITE_DIAG &&
                                (gp_render_layer_raw == 2'd0) &&
                                !gp_tile_hex_mode;
wire obj_line_render_en = debug_object_line_en ||
                          (!ENABLE_OBJ_LINEBUFFER_COMPOSITE &&
                           ENABLE_OBJ_SPRITE_COMPOSITE &&
                           !NO_OBJ_COMPOSITE_DIAG &&
                           (gp_render_layer_raw == 2'd0) &&
                           !gp_tile_hex_mode);

wire obj_lb_line_start = (clkdiv == 4'd0) && (hcnt == 10'd0);
wire obj_lb_display_ready =
    obj_lb_bank_ready[vcnt[OBJ_LB_BANK_BITS-1:0]] &&
    (obj_lb_bank_y[vcnt[OBJ_LB_BANK_BITS-1:0]] == vcnt);
wire obj_lb_cache_scan_start = !obj_lb_cache_scan_active_d &&
                               obj_cache_scan_active;
wire obj_lb_cache_scan_done = obj_lb_cache_scan_active_d &&
                              !obj_cache_scan_active;
wire [8:0] obj_lb_target_ahead = obj_lb_target_y - vcnt;
wire [3:0] obj_lb_target_epoch_next =
    obj_lb_bank_epoch[obj_lb_target_y[OBJ_LB_BANK_BITS-1:0]] + 4'd1;
wire obj_lb_target_bank_free = (vcnt >= V_END) ?
                               (obj_lb_target_y < OBJ_LB_BANK_COUNT) :
                               ((obj_lb_target_y > vcnt) &&
                                (obj_lb_target_ahead < OBJ_LB_BANK_COUNT));
wire obj_lb_can_start_line = obj_lb_frame_active &&
                             (obj_lb_target_y < V_END) &&
                             obj_lb_target_bank_free;
wire obj_lb_active_line_late = obj_lb_line_start &&
                               (vcnt < V_END) &&
                               (obj_lb_state != OBJ_LB_IDLE) &&
                               (obj_lb_target_y <= vcnt);
wire obj_lb_deadline_miss_event = obj_linebuffer_render_en &&
                                  obj_lb_line_start &&
                                  (vcnt < V_END) &&
                                  !obj_lb_display_ready;
wire gp0_obj_cache_we = !cpu_reset_base && obj_cache_scan_active &&
                        (obj_scan_phase == 4'd10) && gp0_obj_stage_store;
wire gp1_obj_cache_we = !cpu_reset_base && obj_cache_scan_active &&
                        (obj_scan_phase == 4'd10) && gp1_obj_stage_store;
wire [OBJ_CACHE_ADDR_W-1:0] gp0_obj_cache_addr = gp0_obj_cache_we ?
    gp0_obj_count : obj_lb_slot;
wire [OBJ_CACHE_ADDR_W-1:0] gp1_obj_cache_addr = gp1_obj_cache_we ?
    gp1_obj_count : obj_lb_slot;
wire [OBJ_CACHE_DATA_W-1:0] gp0_obj_cache_wdata = {
    gp0_obj_stage_draw_x, gp0_obj_stage_draw_y,
    gp0_obj_stage_w, gp0_obj_stage_h,
    gp0_obj_word0, gp0_obj_word0[1:0], gp0_obj_word1
};
wire [OBJ_CACHE_DATA_W-1:0] gp1_obj_cache_wdata = {
    gp1_obj_stage_draw_x, gp1_obj_stage_draw_y,
    gp1_obj_stage_w, gp1_obj_stage_h,
    gp1_obj_word0, gp1_obj_word0[1:0], gp1_obj_word1
};

batsugun_obj_cache_ram #(
    .ADDR_W (OBJ_CACHE_ADDR_W),
    .DATA_W (OBJ_CACHE_DATA_W),
    .DEPTH  (1 << OBJ_CACHE_ADDR_W)
) u_gp0_obj_cache_ram (
    .clk     (clk),
    .addr    (gp0_obj_cache_addr),
    .wr_data (gp0_obj_cache_wdata),
    .wr_en   (gp0_obj_cache_we),
    .rd_data (gp0_obj_cache_q)
);

batsugun_obj_cache_ram #(
    .ADDR_W (OBJ_CACHE_ADDR_W),
    .DATA_W (OBJ_CACHE_DATA_W),
    .DEPTH  (1 << OBJ_CACHE_ADDR_W)
) u_gp1_obj_cache_ram (
    .clk     (clk),
    .addr    (gp1_obj_cache_addr),
    .wr_data (gp1_obj_cache_wdata),
    .wr_en   (gp1_obj_cache_we),
    .rd_data (gp1_obj_cache_q)
);

wire [6:0] obj_lb_selected_count = obj_lb_gp_sel ?
                                   gp1_obj_count : gp0_obj_count;
wire [OBJ_CACHE_DATA_W-1:0] obj_lb_selected_cache_q = obj_lb_gp_sel ?
    gp1_obj_cache_q : gp0_obj_cache_q;
wire [8:0] obj_lb_selected_x = obj_lb_selected_cache_q[67:59];
wire [8:0] obj_lb_selected_y = obj_lb_selected_cache_q[58:50];
wire [7:0] obj_lb_selected_w = obj_lb_selected_cache_q[49:42];
wire [7:0] obj_lb_selected_h = obj_lb_selected_cache_q[41:34];
wire [15:0] obj_lb_selected_attr = obj_lb_selected_cache_q[33:18];
wire [17:0] obj_lb_selected_code = obj_lb_selected_cache_q[17:0];
wire [8:0] obj_lb_selected_dy = obj_lb_target_y - obj_lb_selected_y;
wire obj_lb_selected_on_line =
    obj_lb_selected_dy < {1'b0, obj_lb_selected_h};
wire [4:0] obj_lb_selected_w_tiles = obj_lb_selected_w[7:3];
wire [4:0] obj_lb_selected_h_tiles = obj_lb_selected_h[7:3];
wire [4:0] obj_lb_selected_tile_y = obj_lb_selected_attr[13] ?
    (obj_lb_selected_h_tiles - 5'd1 - {1'b0, obj_lb_selected_dy[6:3]}) :
    {1'b0, obj_lb_selected_dy[6:3]};
wire [2:0] obj_lb_selected_row = obj_lb_selected_attr[13] ?
                                  (3'd7 - obj_lb_selected_dy[2:0]) :
                                  obj_lb_selected_dy[2:0];
wire [4:0] obj_lb_code_tile_x = obj_lb_obj_attr[12] ?
    (obj_lb_w_tiles - 5'd1 - obj_lb_tile_x) : obj_lb_tile_x;
wire [8:0] obj_lb_tile_index = gp_obj_subtile_index_calc(
    obj_lb_w_tiles, obj_lb_code_tile_x, obj_lb_tile_y
);
wire [17:0] obj_lb_tile_code = obj_lb_obj_code +
                                {9'd0, obj_lb_tile_index};
wire [AW-1:0] obj_lb_tile_addr_lo = gp_sprite_gfx_addr_calc(
    obj_lb_gp_sel, obj_lb_tile_code, obj_lb_row,
    gp_live_gfx_map_mode, 1'b0
);
wire [AW-1:0] obj_lb_tile_addr_hi = gp_sprite_gfx_addr_calc(
    obj_lb_gp_sel, obj_lb_tile_code, obj_lb_row,
    gp_live_gfx_map_mode, 1'b1
);
wire [9:0] obj_lb_tile_screen_x_sum = {1'b0, obj_lb_obj_x} +
                                      {2'b00, obj_lb_tile_x, 3'b000};
wire [8:0] obj_lb_tile_screen_x = obj_lb_tile_screen_x_sum[8:0];
wire [9:0] obj_lb_tile_screen_end = {1'b0, obj_lb_tile_screen_x} + 10'd8;
wire obj_lb_tile_visible = (obj_lb_tile_screen_x < H_END[8:0]) ||
                           (obj_lb_tile_screen_end > 10'd512);
wire [9:0] obj_lb_pixel_x_sum = {1'b0, obj_lb_obj_x} +
                                {2'b00, obj_lb_tile_x, 3'b000} +
                                {7'd0, obj_lb_pixel};
wire [8:0] obj_lb_pixel_x = obj_lb_pixel_x_sum[8:0];
wire [2:0] obj_lb_source_x = obj_lb_obj_attr[12] ?
                              (3'd7 - obj_lb_pixel) : obj_lb_pixel;
wire [3:0] obj_lb_pixel_pen = gp_sprite_decode_sample_calc(
    GP_LIVE_PLANE_SWAP ? obj_lb_tile_hi : obj_lb_tile_lo,
    GP_LIVE_PLANE_SWAP ? obj_lb_tile_lo : obj_lb_tile_hi,
    obj_lb_source_x, gp_live_gfx_decode_mode
);
wire [3:0] obj_lb_pixel_pri = obj_lb_obj_attr[11:8];
wire [6:0] obj_lb_pixel_color = {1'b0, obj_lb_obj_attr[7:2]};
wire [15:0] obj_lb_pixel_data = {1'b1, obj_lb_pixel_pri,
                                 obj_lb_pixel_color, obj_lb_pixel_pen};
wire gp0_obj_lb_build_valid =
    (gp0_obj_lb_build_q[19:16] == obj_lb_build_epoch) &&
    gp0_obj_lb_build_q[15];
wire gp1_obj_lb_build_valid =
    (gp1_obj_lb_build_q[19:16] == obj_lb_build_epoch) &&
    gp1_obj_lb_build_q[15];
wire obj_lb_pixel_wins = obj_lb_gp_sel ?
    (!gp1_obj_lb_build_valid ||
     (obj_lb_pixel_pri >= gp1_obj_lb_build_q[14:11])) :
    (!gp0_obj_lb_build_valid ||
     (obj_lb_pixel_pri >= gp0_obj_lb_build_q[14:11]));
wire obj_lb_clear_we = obj_linebuffer_render_en &&
                       (obj_lb_state == OBJ_LB_CLEAR);
wire obj_lb_pixel_we = obj_linebuffer_render_en &&
                       (obj_lb_state == OBJ_LB_PIXEL_WRITE) &&
                       (obj_lb_pixel_x < H_END[8:0]) &&
                       (obj_lb_pixel_pen != 4'h0) && obj_lb_pixel_wins;
wire [8:0] obj_lb_build_x = obj_lb_clear_we ?
                             obj_lb_clear_x : obj_lb_pixel_x;
wire [OBJ_LB_ADDR_BITS-1:0] obj_lb_build_addr =
    {obj_lb_build_bank, obj_lb_build_x};
wire [OBJ_LB_ADDR_BITS-1:0] obj_lb_display_addr =
    {vcnt[OBJ_LB_BANK_BITS-1:0], hcnt[8:0]};
wire [19:0] obj_lb_write_data = obj_lb_clear_we ?
                                20'h00000 :
                                {obj_lb_build_epoch, obj_lb_pixel_data};
wire gp0_obj_lb_we = obj_lb_clear_we ||
                     (obj_lb_pixel_we && !obj_lb_gp_sel);
wire gp1_obj_lb_we = obj_lb_clear_we ||
                     (obj_lb_pixel_we && obj_lb_gp_sel);
wire [3:0] obj_lb_display_epoch =
    obj_lb_bank_epoch[vcnt[OBJ_LB_BANK_BITS-1:0]];
wire gp0_obj_lb_opaque = obj_linebuffer_render_en && gp_tile_area &&
                         obj_lb_display_ready &&
                         (gp0_obj_lb_display_q[19:16] == obj_lb_display_epoch) &&
                         gp0_obj_lb_display_q[15] &&
                         (gp0_obj_lb_display_q[3:0] != 4'h0);
wire gp1_obj_lb_opaque = obj_linebuffer_render_en && gp_tile_area &&
                         obj_lb_display_ready &&
                         (gp1_obj_lb_display_q[19:16] == obj_lb_display_epoch) &&
                         gp1_obj_lb_display_q[15] &&
                         (gp1_obj_lb_display_q[3:0] != 4'h0);
wire [3:0] gp0_obj_lb_pri_eff = OBJ_PRIORITY_EVEN_MASK_TEST ?
                                 (gp0_obj_lb_display_q[14:11] & 4'he) :
                                  gp0_obj_lb_display_q[14:11];
wire [3:0] gp1_obj_lb_pri_eff = OBJ_PRIORITY_EVEN_MASK_TEST ?
                                 (gp1_obj_lb_display_q[14:11] & 4'he) :
                                  gp1_obj_lb_display_q[14:11];
wire [14:0] gp0_obj_lb_pixel = {gp0_obj_lb_pri_eff,
                                gp0_obj_lb_display_q[10:0]};
wire [14:0] gp1_obj_lb_pixel = {gp1_obj_lb_pri_eff,
                                gp1_obj_lb_display_q[10:0]};

batsugun_obj_line_ram #(
    .ADDR_W (OBJ_LB_ADDR_BITS),
    .DEPTH  (OBJ_LB_BANK_COUNT * 512)
) u_gp0_obj_line_ram (
    .clk        (clk),
    .build_addr (obj_lb_build_addr),
    .build_data (obj_lb_write_data),
    .build_we   (gp0_obj_lb_we),
    .build_q    (gp0_obj_lb_build_q),
    .display_addr(obj_lb_display_addr),
    .display_q  (gp0_obj_lb_display_q)
);

batsugun_obj_line_ram #(
    .ADDR_W (OBJ_LB_ADDR_BITS),
    .DEPTH  (OBJ_LB_BANK_COUNT * 512)
) u_gp1_obj_line_ram (
    .clk        (clk),
    .build_addr (obj_lb_build_addr),
    .build_data (obj_lb_write_data),
    .build_we   (gp1_obj_lb_we),
    .build_q    (gp1_obj_lb_build_q),
    .display_addr(obj_lb_display_addr),
    .display_q  (gp1_obj_lb_display_q)
);
wire [3:0] obj_gfx_debug_slot_select = {gp_tile_word_mode, gp_gfx_map_mode};
wire [3:0] obj_gfx_debug_tile_select = 4'd0;
wire [7:0] obj_gfx_debug_tile_page = {obj_gfx_debug_tile_select, 4'd0};
wire       obj_gfx_fetch_gp_sel = obj_gfx_fetch_slot[4];
wire [5:0] obj_gfx_fetch_count = obj_gfx_fetch_gp_sel ? gp1_obj_count : gp0_obj_count;
wire       obj_gfx_fetch_live = ({2'b00, obj_gfx_fetch_slot[3:0]} < obj_gfx_fetch_count);
wire       obj_gfx_fetch_slot_selected = obj_gfx_fetch_slot[3:0] ==
                                         obj_gfx_debug_slot_select;
wire [7:0] obj_gfx_fetch_w_px = obj_gfx_fetch_gp_sel ?
                                gp1_obj_w[obj_gfx_fetch_slot[3:0]] :
                                gp0_obj_w[obj_gfx_fetch_slot[3:0]];
wire [7:0] obj_gfx_fetch_h_px = obj_gfx_fetch_gp_sel ?
                                gp1_obj_h[obj_gfx_fetch_slot[3:0]] :
                                gp0_obj_h[obj_gfx_fetch_slot[3:0]];
wire [4:0] obj_gfx_fetch_w_tiles = obj_gfx_fetch_w_px[7:3];
wire [4:0] obj_gfx_fetch_h_tiles = obj_gfx_fetch_h_px[7:3];
wire [8:0] obj_gfx_fetch_tile_index = {1'b0, obj_gfx_fetch_tile};
wire [8:0] obj_gfx_fetch_tile_count =
    gp_obj_subtile_index_calc(obj_gfx_fetch_w_tiles,
                              obj_gfx_fetch_w_tiles - 5'd1,
                              obj_gfx_fetch_h_tiles - 5'd1) + 9'd1;
wire       obj_gfx_fetch_tile_live = obj_gfx_fetch_live &&
                                     obj_gfx_fetch_slot_selected &&
                                     (obj_gfx_fetch_tile_index <
                                      obj_gfx_fetch_tile_count);
wire [17:0] obj_gfx_fetch_base_code = obj_gfx_fetch_gp_sel ?
                                      gp1_obj_code[obj_gfx_fetch_slot[3:0]] :
                                      gp0_obj_code[obj_gfx_fetch_slot[3:0]];
wire [17:0] obj_gfx_fetch_code = obj_gfx_fetch_base_code +
                                 {9'd0, obj_gfx_fetch_tile_index};
wire [AW-1:0] obj_gfx_fetch_addr_lo =
    gp_sprite_gfx_addr_calc(obj_gfx_fetch_gp_sel, obj_gfx_fetch_code,
                            obj_gfx_fetch_row, gp_live_gfx_map_mode, 1'b0);
wire [AW-1:0] obj_gfx_fetch_addr_hi =
    gp_sprite_gfx_addr_calc(obj_gfx_fetch_gp_sel, obj_gfx_fetch_code,
                            obj_gfx_fetch_row, gp_live_gfx_map_mode, 1'b1);
wire obj_line_word_start = obj_line_render_en && gp_tile_area && (hcnt[2:0] == 3'd0);
wire obj_line_hblank_prefetch_area = obj_line_render_en &&
                                     (hcnt >= H_END) &&
                                     (hcnt < H_TOTAL) &&
                                     (vcnt < (V_END - 9'd1));
wire obj_line_hblank_word_start = obj_line_hblank_prefetch_area &&
                                  (hcnt[2:0] == 3'd0);
wire obj_line_sched_word_start = obj_line_word_start ||
                                 obj_line_hblank_word_start;
wire [9:0] obj_line_hblank_delta = hcnt - H_END;
wire [8:0] obj_line_hblank_target_x = obj_line_hblank_delta[8:0];
wire [8:0] obj_line_next_vcnt = vcnt + 9'd1;
wire [8:0] obj_seq_next_target_x = obj_line_hblank_word_start ?
                                   obj_line_hblank_target_x :
                                   (hcnt[8:0] + OBJ_SEQ_LOOKAHEAD);
wire [8:0] obj_seq_next_target_y = obj_line_hblank_word_start ?
                                   obj_line_next_vcnt : vcnt;
wire obj_seq_next_target_visible = ({1'b0, obj_seq_next_target_x} < H_END) &&
                                   ({1'b0, obj_seq_next_target_y} < V_END);
wire [8:0] obj_seq_after_launch_target_x = obj_seq_pick_target_x + 9'd8;
wire [8:0] obj_seq_after_launch_target_y = obj_seq_pick_target_y;
wire obj_seq_after_launch_visible =
    ({1'b0, obj_seq_after_launch_target_x} < H_END) &&
    ({1'b0, obj_seq_after_launch_target_y} < V_END);
wire obj_seq_empty_pick_advance = obj_seq_pick_ready &&
                                  !obj_seq_pick_valid;
wire [8:0] obj_seq_restart_target_x = obj_seq_empty_pick_advance ?
                                      obj_seq_after_launch_target_x :
                                      obj_seq_next_target_x;
wire [8:0] obj_seq_restart_target_y = obj_seq_empty_pick_advance ?
                                      obj_seq_after_launch_target_y :
                                      obj_seq_next_target_y;
wire obj_seq_restart_target_visible = obj_seq_empty_pick_advance ?
                                      obj_seq_after_launch_visible :
                                      obj_seq_next_target_visible;
wire [AW-1:0] obj_line_pick_addr_lo =
    gp_sprite_gfx_addr_calc(obj_seq_pick_gp_sel, obj_seq_pick_code,
                            obj_seq_pick_row, gp_live_gfx_map_mode, 1'b0);
wire [AW-1:0] obj_line_pick_addr_hi =
    gp_sprite_gfx_addr_calc(obj_seq_pick_gp_sel, obj_seq_pick_code,
                            obj_seq_pick_row, gp_live_gfx_map_mode, 1'b1);
wire [AW-1:0] obj_line_pick_next_addr_lo =
    gp_sprite_gfx_addr_calc(obj_seq_pick_gp_sel, obj_seq_pick_next_code,
                            obj_seq_pick_row, gp_live_gfx_map_mode, 1'b0);
wire [AW-1:0] obj_line_pick_next_addr_hi =
    gp_sprite_gfx_addr_calc(obj_seq_pick_gp_sel, obj_seq_pick_next_code,
                            obj_seq_pick_row, gp_live_gfx_map_mode, 1'b1);
wire [AW-1:0] obj_line_pick2_addr_lo =
    gp_sprite_gfx_addr_calc(obj_seq_pick2_gp_sel, obj_seq_pick2_code,
                            obj_seq_pick2_row, gp_live_gfx_map_mode, 1'b0);
wire [AW-1:0] obj_line_pick2_addr_hi =
    gp_sprite_gfx_addr_calc(obj_seq_pick2_gp_sel, obj_seq_pick2_code,
                            obj_seq_pick2_row, gp_live_gfx_map_mode, 1'b1);
wire [AW-1:0] obj_line_pick2_next_addr_lo =
    gp_sprite_gfx_addr_calc(obj_seq_pick2_gp_sel, obj_seq_pick2_next_code,
                            obj_seq_pick2_row, gp_live_gfx_map_mode, 1'b0);
wire [AW-1:0] obj_line_pick2_next_addr_hi =
    gp_sprite_gfx_addr_calc(obj_seq_pick2_gp_sel, obj_seq_pick2_next_code,
                            obj_seq_pick2_row, gp_live_gfx_map_mode, 1'b1);
wire obj_line_compose_alt = obj_line_req_phase[2];
wire obj_line_compose_span = obj_line_req_phase[1:0] == 2'b11;
wire [15:0] obj_line_compose_tile0_raw_lo = obj_line_compose_span ?
    obj_line_tile0_lo : obj_line_fetch_lo;
wire [15:0] obj_line_compose_tile0_raw_hi = obj_line_compose_span ?
    obj_line_tile0_hi : gfx_obj_slot_dout;
wire [15:0] obj_line_compose_tile1_raw_lo = obj_line_compose_span ?
    obj_line_fetch_lo : 16'h0000;
wire [15:0] obj_line_compose_tile1_raw_hi = obj_line_compose_span ?
    gfx_obj_slot_dout : 16'h0000;
wire [7:0] obj_line_compose_mask = obj_line_compose_alt ?
    obj_line_req_alt_mask : obj_line_req_mask;
wire [3:0] obj_line_compose_bias = obj_line_compose_alt ?
    obj_line_req_alt_bias : obj_line_req_bias;
wire obj_line_compose_flipx = obj_line_compose_alt ?
    obj_line_req_alt_flipx : obj_line_req_flipx;
wire [31:0] obj_line_composed_pixels = gp_obj_word_pixels_calc(
    GP_LIVE_PLANE_SWAP ? obj_line_compose_tile0_raw_hi :
                         obj_line_compose_tile0_raw_lo,
    GP_LIVE_PLANE_SWAP ? obj_line_compose_tile0_raw_lo :
                         obj_line_compose_tile0_raw_hi,
    GP_LIVE_PLANE_SWAP ? obj_line_compose_tile1_raw_hi :
                         obj_line_compose_tile1_raw_lo,
    GP_LIVE_PLANE_SWAP ? obj_line_compose_tile1_raw_lo :
                         obj_line_compose_tile1_raw_hi,
    obj_line_compose_mask, obj_line_compose_bias, obj_line_compose_flipx,
    gp_live_gfx_decode_mode);
wire [3:0] obj_line_primary_sample = obj_line_latched_valid ?
    gp_obj_packed_sample_calc(obj_line_latched_lo, obj_line_latched_hi,
                              hcnt[2:0]) :
    4'h0;
wire [3:0] obj_line_alt_sample = obj_line_latched_alt_valid ?
    gp_obj_packed_sample_calc(obj_line_latched_alt_lo,
                              obj_line_latched_alt_hi, hcnt[2:0]) :
    4'h0;
wire obj_line_use_alt_sample = (obj_line_primary_sample == 4'h0) &&
                               (obj_line_alt_sample != 4'h0);
wire [3:0] obj_line_sprite_sample = obj_line_use_alt_sample ?
                                    obj_line_alt_sample :
                                    obj_line_primary_sample;
wire obj_line_primary_raw_bit = obj_line_latched_valid &&
                                (obj_line_primary_sample != 4'h0);
wire obj_line_alt_raw_bit = obj_line_latched_alt_valid &&
                            (obj_line_alt_sample != 4'h0);
wire obj_line_sprite_raw_bit = obj_line_use_alt_sample ?
                               obj_line_alt_raw_bit :
                               obj_line_primary_raw_bit;
wire [6:0] obj_line_sprite_color = obj_line_use_alt_sample ?
                                   obj_line_latched_alt_color :
                                   obj_line_latched_color;
wire [3:0] obj_line_sprite_pri = obj_line_use_alt_sample ?
                                 obj_line_latched_alt_pri :
                                 obj_line_latched_pri;
wire obj_line_sprite_gp_sel = obj_line_use_alt_sample ?
                              obj_line_latched_alt_gp_sel :
                              obj_line_latched_gp_sel;
wire obj_line_prefetch_match =
    obj_line_prefetch_ready && obj_line_prefetch_valid &&
    (obj_line_prefetch_x == hcnt[8:0]) && (obj_line_prefetch_y == vcnt);
wire obj_line_prefetch1_match =
    obj_line_prefetch1_ready && obj_line_prefetch1_valid &&
    (obj_line_prefetch1_x == hcnt[8:0]) && (obj_line_prefetch1_y == vcnt);
wire obj_line_prefetch2_match =
    obj_line_prefetch2_ready && obj_line_prefetch2_valid &&
    (obj_line_prefetch2_x == hcnt[8:0]) && (obj_line_prefetch2_y == vcnt);
always @* begin
    obj_line_extra_match = 1'b0;
    obj_line_extra_match_idx = 4'd0;
    obj_line_extra_free = 1'b0;
    obj_line_extra_free_idx = 4'd0;
    for (obj_line_extra_comb_i = 0;
         obj_line_extra_comb_i < OBJ_LINE_EXTRA_PREFETCH_SLOTS;
         obj_line_extra_comb_i = obj_line_extra_comb_i + 1) begin
        if (!obj_line_extra_match &&
            obj_line_prefetch_extra_ready[obj_line_extra_comb_i] &&
            obj_line_prefetch_extra_valid[obj_line_extra_comb_i] &&
            (obj_line_prefetch_extra_x[obj_line_extra_comb_i] == hcnt[8:0]) &&
            (obj_line_prefetch_extra_y[obj_line_extra_comb_i] == vcnt)) begin
            obj_line_extra_match = 1'b1;
            obj_line_extra_match_idx = obj_line_extra_comb_i[3:0];
        end
        if (!obj_line_extra_free &&
            !obj_line_prefetch_extra_ready[obj_line_extra_comb_i]) begin
            obj_line_extra_free = 1'b1;
            obj_line_extra_free_idx = obj_line_extra_comb_i[3:0];
        end
    end
end
wire obj_line_prefetch_expired =
    obj_line_prefetch_ready &&
    ((obj_line_prefetch_y != vcnt) ||
     ((obj_line_prefetch_y == vcnt) && (obj_line_prefetch_x < hcnt[8:0])));
wire obj_line_prefetch1_expired =
    obj_line_prefetch1_ready &&
    ((obj_line_prefetch1_y != vcnt) ||
     ((obj_line_prefetch1_y == vcnt) && (obj_line_prefetch1_x < hcnt[8:0])));
wire obj_line_prefetch2_expired =
    obj_line_prefetch2_ready &&
    ((obj_line_prefetch2_y != vcnt) ||
     ((obj_line_prefetch2_y == vcnt) && (obj_line_prefetch2_x < hcnt[8:0])));

always @(posedge clk) begin
    if (video_runtime_reset) begin
        obj_line_sched_word_event <= 1'b0;
        obj_line_word_event <= 1'b0;
        obj_line_prefetch_match_event <= 1'b0;
        obj_line_prefetch1_match_event <= 1'b0;
        obj_line_prefetch2_match_event <= 1'b0;
        obj_line_extra_match_event <= 1'b0;
        obj_line_extra_match_idx_event <= 4'd0;
        obj_line_prefetch_expired_event <= 1'b0;
        obj_line_prefetch1_expired_event <= 1'b0;
        obj_line_prefetch2_expired_event <= 1'b0;
        obj_line_extra_expired_event <=
            {OBJ_LINE_EXTRA_PREFETCH_SLOTS{1'b0}};
    end else begin
        obj_line_sched_word_event <= 1'b0;
        obj_line_word_event <= 1'b0;
        obj_line_prefetch_match_event <= 1'b0;
        obj_line_prefetch1_match_event <= 1'b0;
        obj_line_prefetch2_match_event <= 1'b0;
        obj_line_extra_match_event <= 1'b0;
        obj_line_prefetch_expired_event <= 1'b0;
        obj_line_prefetch1_expired_event <= 1'b0;
        obj_line_prefetch2_expired_event <= 1'b0;
        obj_line_extra_expired_event <=
            {OBJ_LINE_EXTRA_PREFETCH_SLOTS{1'b0}};

        if ((clkdiv == 4'd0) && obj_line_sched_word_start) begin
            obj_line_sched_word_event <= 1'b1;
            obj_line_word_event <= obj_line_word_start;
            obj_line_prefetch_match_event <= obj_line_prefetch_match;
            obj_line_prefetch1_match_event <= obj_line_prefetch1_match;
            obj_line_prefetch2_match_event <= obj_line_prefetch2_match;
            obj_line_extra_match_event <= obj_line_extra_match;
            obj_line_extra_match_idx_event <= obj_line_extra_match_idx;
            obj_line_prefetch_expired_event <= obj_line_prefetch_expired;
            obj_line_prefetch1_expired_event <= obj_line_prefetch1_expired;
            obj_line_prefetch2_expired_event <= obj_line_prefetch2_expired;
            for (obj_line_event_i = 0;
                 obj_line_event_i < OBJ_LINE_EXTRA_PREFETCH_SLOTS;
                 obj_line_event_i = obj_line_event_i + 1) begin
                obj_line_extra_expired_event[obj_line_event_i] <=
                    obj_line_prefetch_extra_ready[obj_line_event_i] &&
                    ((obj_line_prefetch_extra_y[obj_line_event_i] != vcnt) ||
                     ((obj_line_prefetch_extra_y[obj_line_event_i] == vcnt) &&
                      (obj_line_prefetch_extra_x[obj_line_event_i] < hcnt[8:0])));
            end
        end
    end
end
wire obj_line_prefetch_space =
    !obj_line_prefetch_ready || !obj_line_prefetch1_ready ||
    !obj_line_prefetch2_ready || obj_line_extra_free;
wire obj_bank_gp0_en;
wire obj_bank_gp1_en;
wire       obj_seq_scan_gp_sel = obj_seq_scan_idx[5];
wire [4:0] obj_seq_scan_slot = obj_seq_scan_idx[4:0];
wire [6:0] obj_seq_scan_count_full = obj_seq_scan_gp_sel ? gp1_obj_count :
                                                            gp0_obj_count;
wire [5:0] obj_seq_scan_count = (obj_seq_scan_count_full > OBJ_DEBUG_COUNT) ?
                                 OBJ_DEBUG_COUNT :
                                 obj_seq_scan_count_full[5:0];
wire       obj_seq_scan_bank_en = obj_seq_scan_gp_sel ? obj_bank_gp1_en :
                                                        obj_bank_gp0_en;
wire       obj_seq_scan_count_exhausted =
    {1'b0, obj_seq_scan_slot} >= obj_seq_scan_count;
wire       obj_seq_scan_gp0_exhausted = !obj_seq_scan_gp_sel &&
                                        obj_seq_scan_count_exhausted;
wire       obj_seq_scan_gp1_exhausted = obj_seq_scan_gp_sel &&
                                        obj_seq_scan_count_exhausted;
wire       obj_seq_scan_live = obj_seq_scan_bank_en &&
                               ({1'b0, obj_seq_scan_slot} < obj_seq_scan_count);
wire [7:0] obj_seq_scan_w = obj_seq_scan_gp_sel ?
                            gp1_obj_w[obj_seq_scan_slot[3:0]] :
                            gp0_obj_w[obj_seq_scan_slot[3:0]];
wire [7:0] obj_seq_scan_h = obj_seq_scan_gp_sel ?
                            gp1_obj_h[obj_seq_scan_slot[3:0]] :
                            gp0_obj_h[obj_seq_scan_slot[3:0]];
wire [15:0] obj_seq_scan_attr = obj_seq_scan_gp_sel ?
                                gp1_obj_attr[obj_seq_scan_slot[3:0]] :
                                gp0_obj_attr[obj_seq_scan_slot[3:0]];
wire [17:0] obj_seq_scan_code_base = obj_seq_scan_gp_sel ?
                                     gp1_obj_code[obj_seq_scan_slot[3:0]] :
                                     gp0_obj_code[obj_seq_scan_slot[3:0]];
wire [8:0] obj_seq_scan_x = obj_seq_scan_gp_sel ?
                            gp1_obj_x[obj_seq_scan_slot[3:0]] :
                            gp0_obj_x[obj_seq_scan_slot[3:0]];
wire [8:0] obj_seq_scan_y = obj_seq_scan_gp_sel ?
                            gp1_obj_y[obj_seq_scan_slot[3:0]] :
                            gp0_obj_y[obj_seq_scan_slot[3:0]];
wire [8:0] obj_seq_scan_dx = obj_seq_target_x - obj_seq_scan_x;
wire [8:0] obj_seq_scan_dy = obj_seq_target_y - obj_seq_scan_y;
wire [4:0] obj_seq_scan_w_tiles = obj_seq_scan_w[7:3];
wire [4:0] obj_seq_scan_h_tiles = obj_seq_scan_h[7:3];
wire [7:0] obj_seq_scan_word_mask =
    gp_obj_word_mask_calc(obj_seq_scan_dx, obj_seq_scan_w);
wire       obj_seq_scan_hit = obj_seq_scan_active && obj_seq_scan_live &&
                              (|obj_seq_scan_word_mask) &&
                              (obj_seq_scan_dy < {1'b0, obj_seq_scan_h});
wire [7:0] obj_seq_eval_width_px = {obj_seq_eval_w_tiles, 3'b000};
wire       obj_seq_eval_target_inside =
    obj_seq_eval_dx < {1'b0, obj_seq_eval_width_px};
wire [8:0] obj_seq_eval_lead = 9'd0 - obj_seq_eval_dx;
wire [4:0] obj_seq_eval_logical_tile_x = obj_seq_eval_target_inside ?
    {1'b0, obj_seq_eval_dx[6:3]} : 5'd0;
wire [4:0] obj_seq_eval_next_logical_tile_x =
    obj_seq_eval_logical_tile_x + 5'd1;
wire [7:0] obj_seq_eval_word_mask =
    gp_obj_word_mask_calc(obj_seq_eval_dx, obj_seq_eval_width_px);
wire [3:0] obj_seq_eval_source_bias = obj_seq_eval_target_inside ?
    {1'b0, obj_seq_eval_dx[2:0]} :
    (4'd0 - {1'b0, obj_seq_eval_lead[2:0]});
wire       obj_seq_eval_next_valid = obj_seq_eval_target_inside &&
    (obj_seq_eval_dx[2:0] != 3'd0) &&
    (obj_seq_eval_next_logical_tile_x < obj_seq_eval_w_tiles);
wire [4:0] obj_seq_eval_tile_x = obj_seq_eval_attr[12] ?
                                 (obj_seq_eval_w_tiles - 5'd1 -
                                  obj_seq_eval_logical_tile_x) :
                                 obj_seq_eval_logical_tile_x;
wire [4:0] obj_seq_eval_next_tile_x = obj_seq_eval_attr[12] ?
    (obj_seq_eval_w_tiles - 5'd1 - obj_seq_eval_next_logical_tile_x) :
    obj_seq_eval_next_logical_tile_x;
wire [4:0] obj_seq_eval_tile_y = obj_seq_eval_attr[13] ?
                                 (obj_seq_eval_h_tiles - 5'd1 -
                                  {1'b0, obj_seq_eval_dy[6:3]}) :
                                 {1'b0, obj_seq_eval_dy[6:3]};
wire [8:0] obj_seq_eval_tile_index =
    gp_obj_subtile_index_calc(obj_seq_eval_w_tiles,
                              obj_seq_eval_tile_x,
                              obj_seq_eval_tile_y);
wire [8:0] obj_seq_eval_next_tile_index =
    gp_obj_subtile_index_calc(obj_seq_eval_w_tiles,
                              obj_seq_eval_next_tile_x,
                              obj_seq_eval_tile_y);
wire [17:0] obj_seq_eval_code = obj_seq_eval_code_base +
                                {9'd0, obj_seq_eval_tile_index};
wire [17:0] obj_seq_eval_next_code = obj_seq_eval_code_base +
                                     {9'd0, obj_seq_eval_next_tile_index};
wire [2:0] obj_seq_eval_row = obj_seq_eval_attr[13] ?
                              (3'd7 - obj_seq_eval_dy[2:0]) :
                              obj_seq_eval_dy[2:0];
wire       obj_seq_eval_wins_primary =
    obj_seq_eval_valid &&
    ((!obj_seq_pick_valid) ||
     (obj_seq_eval_attr[11:8] >= obj_seq_pick_pri));
wire       obj_seq_eval_wins_secondary =
    obj_seq_eval_valid && !obj_seq_eval_wins_primary &&
    ((!obj_seq_pick2_valid) ||
     (obj_seq_eval_attr[11:8] >= obj_seq_pick2_pri));
wire       obj_seq_pick_launchable = !obj_line_req_pending &&
                                     obj_line_prefetch_space &&
                                     obj_seq_pick_ready &&
                                     obj_seq_pick_valid;
wire       obj_meta_gp_sel = obj_gfx_debug_slot_valid ? obj_gfx_debug_slot[4] :
                                                        gp_render_sel;
wire [3:0] obj_meta_slot = obj_gfx_debug_slot_valid ? obj_gfx_debug_slot[3:0] :
                                                      obj_gfx_debug_slot_select;
wire [15:0] obj_meta_attr = obj_meta_gp_sel ? gp1_obj_attr[obj_meta_slot] :
                                              gp0_obj_attr[obj_meta_slot];
wire [17:0] obj_meta_code = obj_meta_gp_sel ? gp1_obj_code[obj_meta_slot] :
                                              gp0_obj_code[obj_meta_slot];
wire [15:0] obj_meta_raw_x = obj_meta_gp_sel ? gp1_obj_raw_x[obj_meta_slot] :
                                               gp0_obj_raw_x[obj_meta_slot];
wire [15:0] obj_meta_raw_y = obj_meta_gp_sel ? gp1_obj_raw_y[obj_meta_slot] :
                                               gp0_obj_raw_y[obj_meta_slot];
wire [7:0] obj_meta_w = obj_meta_gp_sel ? gp1_obj_w[obj_meta_slot] :
                                          gp0_obj_w[obj_meta_slot];
wire [7:0] obj_meta_h = obj_meta_gp_sel ? gp1_obj_h[obj_meta_slot] :
                                          gp0_obj_h[obj_meta_slot];
wire [15:0] obj_meta_counts = {gp0_obj_count[5:0],
                               gp1_obj_count[5:0], obj_meta_slot};
wire gp_tile_hex_code_mode = gp_tile_hex_mode && (gp_gfx_decode_mode == 2'd0);
wire gp_rom_probe_mode = gp_tile_hex_mode && (gp_gfx_decode_mode != 2'd0);
wire gp_composite_mode = gp_render_layer_raw == 2'd0;
wire [1:0] gp_render_layer = gp_composite_mode ? 2'd0 : (gp_render_layer_raw - 2'd1);
wire [5:0] gp_debug_layer_mask = gp_render_sel ?
                                 ((gp_render_layer == 2'd0) ? 6'b001000 :
                                  (gp_render_layer == 2'd1) ? 6'b010000 :
                                                               6'b100000) :
                                 ((gp_render_layer == 2'd0) ? 6'b000001 :
                                  (gp_render_layer == 2'd1) ? 6'b000010 :
                                                               6'b000100);
wire [5:0] gp_banked_layer_mask = comp_frame_bank ? 6'b111000 : 6'b000111;
wire [5:0] comp_layer_fetch_en = gp_composite_mode ?
                                                     (COMP_BANKED_SCROLL_DIAG ?
                                                      gp_banked_layer_mask :
                                                      6'b111111) :
                                                     gp_debug_layer_mask;
wire obj_bank_diag_en = COMP_BANK_OBJECTS_DIAG && COMP_BANKED_SCROLL_DIAG &&
                        gp_composite_mode;
assign obj_bank_gp0_en = !obj_bank_diag_en || !comp_frame_bank;
assign obj_bank_gp1_en = !obj_bank_diag_en || comp_frame_bank;
// MAME's original-board origins are +42/+40/+38 X and +17 Y modulo 512.
// The staged compositor fetch is one eight-pixel word ahead of the sampled
// word, so its X coordinate also includes that word-phase correction.
localparam [8:0] GP_LAYER0_X_ADD = 9'h032;
localparam [8:0] GP_LAYER1_X_ADD = 9'h030;
localparam [8:0] GP_LAYER2_X_ADD = 9'h02e;
localparam [8:0] GP_LAYER_Y_ADD  = 9'h011;
wire [15:0] gp0_layer_scroll_x = (gp_render_layer == 2'd0) ? gp0_scroll0_px :
                                  (gp_render_layer == 2'd1) ? gp0_scroll2_px :
                                                               gp0_scroll4_px;
wire [15:0] gp0_layer_scroll_y = (gp_render_layer == 2'd0) ? gp0_scroll1_px :
                                  (gp_render_layer == 2'd1) ? gp0_scroll3_px :
                                                               gp0_scroll5_px;
wire [15:0] gp1_layer_scroll_x = (gp_render_layer == 2'd0) ? gp1_scroll0_px :
                                  (gp_render_layer == 2'd1) ? gp1_scroll2_px :
                                                               gp1_scroll4_px;
wire [15:0] gp1_layer_scroll_y = (gp_render_layer == 2'd0) ? gp1_scroll1_px :
                                  (gp_render_layer == 2'd1) ? gp1_scroll3_px :
                                                               gp1_scroll5_px;
wire [15:0] gp_layer_scroll_x = gp_render_sel ? gp1_layer_scroll_x : gp0_layer_scroll_x;
wire [15:0] gp_layer_scroll_y = gp_render_sel ? gp1_layer_scroll_y : gp0_layer_scroll_y;
wire [8:0] gp_layer_x_add = (gp_render_layer == 2'd0) ? GP_LAYER0_X_ADD :
                             (gp_render_layer == 2'd1) ? GP_LAYER1_X_ADD :
                                                          GP_LAYER2_X_ADD;
wire [8:0] gp_tile_x_mod = hcnt[8:0] + gp_layer_scroll_x[8:0] + gp_layer_x_add;
wire [8:0] gp_tile_y_mod = vcnt + gp_layer_scroll_y[8:0] + GP_LAYER_Y_ADD;
wire [9:0] gp_tile_x = {1'b0, gp_tile_x_mod};
wire [8:0] gp_tile_y = gp_tile_y_mod;
// MAME models each GP9001 scroll plane as a 32x32 map of 16x16 tiles,
// with two 16-bit words per tile.
wire [4:0] gp_tile_row = gp_tile_y[8:4];
wire [4:0] gp_tile_col = gp_tile_x[8:4];
wire [4:0] gp_hex_tile_col = {gp_tile_x[8:5], 1'b0};
wire [12:0] gp_tile_base = (gp_render_layer == 2'd0) ? 13'h0000 :
                           (gp_render_layer == 2'd1) ? 13'h0800 :
                                                        13'h1000;
reg  [12:0] gp_render_scan_addr = 13'h0000;
reg  [15:0] gp_tile_attr = 16'h0000;
reg  [15:0] gp_tile_code = 16'h0000;
wire [15:0] gp_render_scan_dout = gp_render_sel ? gp1_scan_dout : gp0_scan_dout;
wire [12:0] gp0_scan_addr = gp_tile_hex_mode ? (gp_render_sel ? gp0_sample_addr : gp_render_scan_addr) :
                                                gp0_comp_scan_addr;
wire [12:0] gp1_scan_addr = gp_tile_hex_mode ? (gp_render_sel ? gp_render_scan_addr : gp1_sample_addr) :
                                                gp1_comp_scan_addr;
wire [3:0] gp_gfx_px = gp_tile_x[3:0];
wire [3:0] gp_gfx_py = gp_tile_y[3:0];
wire [8:0] gp_fetch_tile_x_mod = gp_tile_x_mod + 9'd8;
wire [9:0] gp_fetch_tile_x = {1'b0, gp_fetch_tile_x_mod};
wire [4:0] gp_fetch_tile_col = gp_fetch_tile_x[8:4];
wire [12:0] gp_tile_pair_addr = gp_tile_base + {2'b00, gp_tile_row, gp_tile_col, 1'b0};
wire [12:0] gp_hex_tile_pair_addr = gp_tile_base + {2'b00, gp_tile_row, gp_hex_tile_col, 1'b0};
wire [12:0] gp_hex_tile_attr_addr = gp_hex_tile_pair_addr;
wire [12:0] gp_hex_tile_code_addr = gp_hex_tile_pair_addr + 13'd1;
wire [12:0] gp_fetch_tile_pair_addr = gp_tile_base + {2'b00, gp_tile_row, gp_fetch_tile_col, 1'b0};
wire [12:0] gp_fetch_tile_attr_addr = gp_fetch_tile_pair_addr;
wire [12:0] gp_fetch_tile_code_addr = gp_fetch_tile_pair_addr + 13'd1;
wire [12:0] gp_scan_tile_attr_addr = gp_tile_hex_code_mode ? gp_hex_tile_attr_addr : gp_fetch_tile_attr_addr;
wire [12:0] gp_scan_tile_code_addr = gp_tile_hex_code_mode ? gp_hex_tile_code_addr : gp_fetch_tile_code_addr;
wire [15:0] gp_tile_attr_pair = gp_tile_word_mode[1] ? gp_tile_code : gp_tile_attr;
wire [15:0] gp_tile_code_pair = gp_tile_word_mode[1] ? gp_tile_attr : gp_tile_code;
wire [15:0] gp_tile_attr_eff = gp_tile_word_mode[0] ? {gp_tile_attr_pair[7:0], gp_tile_attr_pair[15:8]} :
                                                        gp_tile_attr_pair;
wire [15:0] gp_tile_code_eff = gp_tile_word_mode[0] ? {gp_tile_code_pair[7:0], gp_tile_code_pair[15:8]} :
                                                        gp_tile_code_pair;
wire gp_tile_nonzero = gp_tile_code_eff != 16'h0000;

wire gp_comp_prefetch_hblank = hcnt >= H_PREFETCH_START;
wire [8:0] gp_comp_next_vcnt = (vcnt == (V_TOTAL - 9'd1)) ? 9'd0 : (vcnt + 9'd1);
wire [8:0] gp_comp_hcnt = gp_comp_prefetch_hblank ?
                           (hcnt[8:0] + GP_HBLANK_PREFETCH_X_ADD) : hcnt[8:0];
wire [8:0] gp_comp_vcnt = gp_comp_prefetch_hblank ? gp_comp_next_vcnt : vcnt;
wire [8:0] gp_comp_fetch_target_x = gp_comp_hcnt + 9'd24;
wire gp_comp_area = !gp_tile_hex_mode &&
                    (gp_tile_area ||
                     (gp_comp_prefetch_hblank &&
                      (gp_comp_next_vcnt >= V_START) &&
                      (gp_comp_next_vcnt < V_END)));
wire [8:0] gp0_l0_x_mod = gp_comp_hcnt + gp0_scroll0_px[8:0] + GP_LAYER0_X_ADD;
wire [8:0] gp0_l1_x_mod = gp_comp_hcnt + gp0_scroll2_px[8:0] + GP_LAYER1_X_ADD;
wire [8:0] gp0_l2_x_mod = gp_comp_hcnt + gp0_scroll4_px[8:0] + GP_LAYER2_X_ADD;
wire [8:0] gp1_l0_x_mod = gp_comp_hcnt + gp1_scroll0_px[8:0] + GP_LAYER0_X_ADD;
wire [8:0] gp1_l1_x_mod = gp_comp_hcnt + gp1_scroll2_px[8:0] + GP_LAYER1_X_ADD;
wire [8:0] gp1_l2_x_mod = gp_comp_hcnt + gp1_scroll4_px[8:0] + GP_LAYER2_X_ADD;
wire [8:0] gp0_l0_y_mod = gp_comp_vcnt + gp0_scroll1_px[8:0] + GP_LAYER_Y_ADD;
wire [8:0] gp0_l1_y_mod = gp_comp_vcnt + gp0_scroll3_px[8:0] + GP_LAYER_Y_ADD;
wire [8:0] gp0_l2_y_mod = gp_comp_vcnt + gp0_scroll5_px[8:0] + GP_LAYER_Y_ADD;
wire [8:0] gp1_l0_y_mod = gp_comp_vcnt + gp1_scroll1_px[8:0] + GP_LAYER_Y_ADD;
wire [8:0] gp1_l1_y_mod = gp_comp_vcnt + gp1_scroll3_px[8:0] + GP_LAYER_Y_ADD;
wire [8:0] gp1_l2_y_mod = gp_comp_vcnt + gp1_scroll5_px[8:0] + GP_LAYER_Y_ADD;
wire [8:0] gp0_l0_fetch_x_mod = gp0_l0_x_mod + 9'd16;
wire [8:0] gp0_l1_fetch_x_mod = gp0_l1_x_mod + 9'd16;
wire [8:0] gp0_l2_fetch_x_mod = gp0_l2_x_mod + 9'd16;
wire [8:0] gp1_l0_fetch_x_mod = gp1_l0_x_mod + 9'd16;
wire [8:0] gp1_l1_fetch_x_mod = gp1_l1_x_mod + 9'd16;
wire [8:0] gp1_l2_fetch_x_mod = gp1_l2_x_mod + 9'd16;
wire [12:0] gp0_l0_pair_addr = gp_tile_pair_addr_calc(2'd0, gp0_l0_y_mod[8:4], gp0_l0_fetch_x_mod[8:4]);
wire [12:0] gp0_l1_pair_addr = gp_tile_pair_addr_calc(2'd1, gp0_l1_y_mod[8:4], gp0_l1_fetch_x_mod[8:4]);
wire [12:0] gp0_l2_pair_addr = gp_tile_pair_addr_calc(2'd2, gp0_l2_y_mod[8:4], gp0_l2_fetch_x_mod[8:4]);
wire [12:0] gp1_l0_pair_addr = gp_tile_pair_addr_calc(2'd0, gp1_l0_y_mod[8:4], gp1_l0_fetch_x_mod[8:4]);
wire [12:0] gp1_l1_pair_addr = gp_tile_pair_addr_calc(2'd1, gp1_l1_y_mod[8:4], gp1_l1_fetch_x_mod[8:4]);
wire [12:0] gp1_l2_pair_addr = gp_tile_pair_addr_calc(2'd2, gp1_l2_y_mod[8:4], gp1_l2_fetch_x_mod[8:4]);
wire [4:0] gp0_l0_word_in_tile = gp_word_in_tile_calc(gp0_l0_y_mod[3:0], gp0_l0_fetch_x_mod[3]);
wire [4:0] gp0_l1_word_in_tile = gp_word_in_tile_calc(gp0_l1_y_mod[3:0], gp0_l1_fetch_x_mod[3]);
wire [4:0] gp0_l2_word_in_tile = gp_word_in_tile_calc(gp0_l2_y_mod[3:0], gp0_l2_fetch_x_mod[3]);
wire [4:0] gp1_l0_word_in_tile = gp_word_in_tile_calc(gp1_l0_y_mod[3:0], gp1_l0_fetch_x_mod[3]);
wire [4:0] gp1_l1_word_in_tile = gp_word_in_tile_calc(gp1_l1_y_mod[3:0], gp1_l1_fetch_x_mod[3]);
wire [4:0] gp1_l2_word_in_tile = gp_word_in_tile_calc(gp1_l2_y_mod[3:0], gp1_l2_fetch_x_mod[3]);
wire gp0_l0_word_start = gp_comp_area && (gp0_l0_x_mod[2:0] == 3'd0);
wire gp0_l1_word_start = gp_comp_area && (gp0_l1_x_mod[2:0] == 3'd0);
wire gp0_l2_word_start = gp_comp_area && (gp0_l2_x_mod[2:0] == 3'd0);
wire gp1_l0_word_start = gp_comp_area && (gp1_l0_x_mod[2:0] == 3'd0);
wire gp1_l1_word_start = gp_comp_area && (gp1_l1_x_mod[2:0] == 3'd0);
wire gp1_l2_word_start = gp_comp_area && (gp1_l2_x_mod[2:0] == 3'd0);
wire [4:0] comp_word_cache_idx0 = {3'd0, gp_comp_hcnt[4:3]};
wire [4:0] comp_word_cache_idx1 = {3'd1, gp_comp_hcnt[4:3]};
wire [4:0] comp_word_cache_idx2 = {3'd2, gp_comp_hcnt[4:3]};
wire [4:0] comp_word_cache_idx3 = {3'd3, gp_comp_hcnt[4:3]};
wire [4:0] comp_word_cache_idx4 = {3'd4, gp_comp_hcnt[4:3]};
wire [4:0] comp_word_cache_idx5 = {3'd5, gp_comp_hcnt[4:3]};
wire [4:0] comp_word_cache_fetch_idx0 = {3'd0, gp_comp_fetch_target_x[4:3]};
wire [4:0] comp_word_cache_fetch_idx1 = {3'd1, gp_comp_fetch_target_x[4:3]};
wire [4:0] comp_word_cache_fetch_idx2 = {3'd2, gp_comp_fetch_target_x[4:3]};
wire [4:0] comp_word_cache_fetch_idx3 = {3'd3, gp_comp_fetch_target_x[4:3]};
wire [4:0] comp_word_cache_fetch_idx4 = {3'd4, gp_comp_fetch_target_x[4:3]};
wire [4:0] comp_word_cache_fetch_idx5 = {3'd5, gp_comp_fetch_target_x[4:3]};
wire [5:0] comp_prefetch_ready = {
    comp_word_cache_ready[comp_word_cache_idx5] &&
        (comp_word_cache_target_x[comp_word_cache_idx5] == gp_comp_hcnt) &&
        (comp_word_cache_target_y[comp_word_cache_idx5] == gp_comp_vcnt),
    comp_word_cache_ready[comp_word_cache_idx4] &&
        (comp_word_cache_target_x[comp_word_cache_idx4] == gp_comp_hcnt) &&
        (comp_word_cache_target_y[comp_word_cache_idx4] == gp_comp_vcnt),
    comp_word_cache_ready[comp_word_cache_idx3] &&
        (comp_word_cache_target_x[comp_word_cache_idx3] == gp_comp_hcnt) &&
        (comp_word_cache_target_y[comp_word_cache_idx3] == gp_comp_vcnt),
    comp_word_cache_ready[comp_word_cache_idx2] &&
        (comp_word_cache_target_x[comp_word_cache_idx2] == gp_comp_hcnt) &&
        (comp_word_cache_target_y[comp_word_cache_idx2] == gp_comp_vcnt),
    comp_word_cache_ready[comp_word_cache_idx1] &&
        (comp_word_cache_target_x[comp_word_cache_idx1] == gp_comp_hcnt) &&
        (comp_word_cache_target_y[comp_word_cache_idx1] == gp_comp_vcnt),
    comp_word_cache_ready[comp_word_cache_idx0] &&
        (comp_word_cache_target_x[comp_word_cache_idx0] == gp_comp_hcnt) &&
        (comp_word_cache_target_y[comp_word_cache_idx0] == gp_comp_vcnt)
};
wire [5:0] comp_prefetch_valid = {
    comp_word_cache_valid[comp_word_cache_idx5],
    comp_word_cache_valid[comp_word_cache_idx4],
    comp_word_cache_valid[comp_word_cache_idx3],
    comp_word_cache_valid[comp_word_cache_idx2],
    comp_word_cache_valid[comp_word_cache_idx1],
    comp_word_cache_valid[comp_word_cache_idx0]
};
wire [5:0] comp_word_start_mask = {
    gp1_l2_word_start,
    gp1_l1_word_start,
    gp1_l0_word_start,
    gp0_l2_word_start,
    gp0_l1_word_start,
    gp0_l0_word_start
};
wire [5:0] comp_word_miss_mask = {6{(clkdiv == 4'd1) && gp_tile_area}} &
                                  comp_word_start_event &
                                  comp_layer_fetch_en &
                                  ~comp_prefetch_ready;
wire gfx_req_target_due = comp_word_start_event[gfx_req_slot] &&
                          (gfx_req_target_x == gp_comp_hcnt) &&
                          (gfx_req_target_y == gp_comp_vcnt);
wire gfx_req_deadline_miss =
    (clkdiv == 4'd1) && gfx_req_pending && (gfx_req_slot < 3'd6) &&
    (gfx_req_target_due ||
     (!gfx_req_stage && !gfx_req_far_stage && !gfx_req_deep_stage &&
      comp_word_start_event[gfx_req_slot]));
wire gfx_req_probe_target_due = comp_word_start_event[gfx_req_probe_slot] &&
                                (gfx_req_probe_target_x == gp_comp_hcnt) &&
                                (gfx_req_probe_target_y == gp_comp_vcnt);
wire gfx_req_probe_deadline_miss =
    (clkdiv == 4'd1) && gfx_req_probe_pending && (gfx_req_probe_slot < 3'd6) &&
    (gfx_req_probe_target_due ||
     (!gfx_req_probe_stage && !gfx_req_probe_far_stage &&
      !gfx_req_probe_deep_stage &&
      comp_word_start_event[gfx_req_probe_slot]));
wire gfx_req_word_start = (clkdiv == 4'd1) &&
                          (gfx_req_slot < 3'd6) &&
                          comp_word_start_event[gfx_req_slot];
wire gfx_req_probe_word_start = (clkdiv == 4'd1) &&
                                (gfx_req_probe_slot < 3'd6) &&
                                comp_word_start_event[gfx_req_probe_slot];
wire gfx_req_store_deep_stage = gfx_req_deep_stage && !gfx_req_word_start;
wire gfx_req_store_far_stage = (gfx_req_far_stage && !gfx_req_word_start) ||
                               (gfx_req_deep_stage && gfx_req_word_start);
wire gfx_req_store_stage = gfx_req_stage &&
                           !gfx_req_word_start;
wire gfx_req_store_next_stage = gfx_req_store_stage ||
                                (gfx_req_far_stage && gfx_req_word_start);
wire gfx_req_probe_store_far_stage = gfx_req_probe_far_stage &&
                                     !gfx_req_probe_word_start;
wire gfx_req_probe_store_deep_stage = gfx_req_probe_deep_stage &&
                                      !gfx_req_probe_word_start;
wire gfx_req_probe_store_far_eff_stage =
    gfx_req_probe_store_far_stage ||
    (gfx_req_probe_deep_stage && gfx_req_probe_word_start);
wire gfx_req_probe_store_stage = gfx_req_probe_stage &&
                                 !gfx_req_probe_word_start;
wire gfx_req_probe_store_next_stage = gfx_req_probe_store_stage ||
                                      (gfx_req_probe_far_stage &&
                                       gfx_req_probe_word_start);
wire [5:0] comp_fetch_pending_eff = comp_fetch_pending & comp_layer_fetch_en;
wire [5:0] comp_fetch_pending_scroll_eff = comp_fetch_pending_eff;
wire gfx_scroll_req_blocked = obj_gfx_req_pending || gp_rom_probe_mode;
wire [3:0] comp_deadline_raw0 = comp_phase_deadline(gp0_l0_x_mod[2:0]);
wire [3:0] comp_deadline_raw1 = comp_phase_deadline(gp0_l1_x_mod[2:0]);
wire [3:0] comp_deadline_raw2 = comp_phase_deadline(gp0_l2_x_mod[2:0]);
wire [3:0] comp_deadline_raw3 = comp_phase_deadline(gp1_l0_x_mod[2:0]);
wire [3:0] comp_deadline_raw4 = comp_phase_deadline(gp1_l1_x_mod[2:0]);
wire [3:0] comp_deadline_raw5 = comp_phase_deadline(gp1_l2_x_mod[2:0]);
wire [3:0] comp_deadline0 = comp_fetch_deep_stage[0] ? 4'd15 :
                             (comp_fetch_far_stage[0] ? 4'd15 :
                              (comp_fetch_stage[0] ? 4'd12 : comp_deadline_raw0));
wire [3:0] comp_deadline1 = comp_fetch_deep_stage[1] ? 4'd15 :
                             (comp_fetch_far_stage[1] ? 4'd15 :
                              (comp_fetch_stage[1] ? 4'd12 : comp_deadline_raw1));
wire [3:0] comp_deadline2 = comp_fetch_deep_stage[2] ? 4'd15 :
                             (comp_fetch_far_stage[2] ? 4'd15 :
                              (comp_fetch_stage[2] ? 4'd12 : comp_deadline_raw2));
wire [3:0] comp_deadline3 = comp_fetch_deep_stage[3] ? 4'd15 :
                             (comp_fetch_far_stage[3] ? 4'd15 :
                              (comp_fetch_stage[3] ? 4'd12 : comp_deadline_raw3));
wire [3:0] comp_deadline4 = comp_fetch_deep_stage[4] ? 4'd15 :
                             (comp_fetch_far_stage[4] ? 4'd15 :
                              (comp_fetch_stage[4] ? 4'd12 : comp_deadline_raw4));
wire [3:0] comp_deadline5 = comp_fetch_deep_stage[5] ? 4'd15 :
                             (comp_fetch_far_stage[5] ? 4'd15 :
                              (comp_fetch_stage[5] ? 4'd12 : comp_deadline_raw5));
wire [2:0] comp_fetch_primary_pending = comp_fetch_pending_scroll_eff[2:0];
wire [2:0] comp_fetch_probe_pending = comp_fetch_pending_scroll_eff[5:3];
wire [1:0] comp_fetch_pick_local = gp_comp_prefetch_hblank ?
    (comp_fetch_primary_pending[2] ? 2'd2 :
     comp_fetch_primary_pending[1] ? 2'd1 : 2'd0) :
    comp_fetch_deadline_pick3(
        comp_fetch_primary_pending,
        comp_fetch_rr[1:0],
        comp_deadline0,
        comp_deadline1,
        comp_deadline2
    );
wire [2:0] comp_fetch_pick_slot = {1'b0, comp_fetch_pick_local};
wire comp_fetch_pick_valid = |comp_fetch_primary_pending;
wire comp_fetch_primary_can_grant = comp_fetch_pick_valid &&
                                    !comp_fetch_grant_valid &&
                                    !gfx_req_pending &&
                                    !gfx_scroll_req_blocked;
wire [1:0] comp_fetch_probe_pick_local = gp_comp_prefetch_hblank ?
    (comp_fetch_probe_pending[2] ? 2'd2 :
     comp_fetch_probe_pending[1] ? 2'd1 : 2'd0) :
    comp_fetch_deadline_pick3(
        comp_fetch_probe_pending,
        comp_fetch_probe_rr - 3'd3,
        comp_deadline3,
        comp_deadline4,
        comp_deadline5
    );
wire [2:0] comp_fetch_probe_pick_slot =
    3'd3 + {1'b0, comp_fetch_probe_pick_local};
wire comp_fetch_probe_pick_valid = |comp_fetch_probe_pending;
wire comp_fetch_probe_can_grant = !comp_fetch_probe_grant_valid &&
    !gfx_req_probe_pending &&
    !gfx_scroll_req_blocked && !gp_rom_probe_mode &&
    comp_fetch_probe_pick_valid;
wire comp_fetch_grant_target_due =
    comp_word_start_event[comp_fetch_grant_slot] &&
    (comp_fetch_target_x[comp_fetch_grant_slot] == gp_comp_hcnt) &&
    (comp_fetch_target_y[comp_fetch_grant_slot] == gp_comp_vcnt);
wire comp_fetch_grant_deadline_miss = comp_fetch_grant_valid &&
    (clkdiv == 4'd1) &&
    (comp_fetch_grant_target_due ||
     (!comp_fetch_stage[comp_fetch_grant_slot] &&
      !comp_fetch_far_stage[comp_fetch_grant_slot] &&
      !comp_fetch_deep_stage[comp_fetch_grant_slot] &&
      comp_word_start_event[comp_fetch_grant_slot]));
wire comp_fetch_probe_grant_target_due =
    comp_word_start_event[comp_fetch_probe_grant_slot] &&
    (comp_fetch_target_x[comp_fetch_probe_grant_slot] == gp_comp_hcnt) &&
    (comp_fetch_target_y[comp_fetch_probe_grant_slot] == gp_comp_vcnt);
wire comp_fetch_probe_grant_deadline_miss = comp_fetch_probe_grant_valid &&
    (clkdiv == 4'd1) &&
    (comp_fetch_probe_grant_target_due ||
     (!comp_fetch_stage[comp_fetch_probe_grant_slot] &&
      !comp_fetch_far_stage[comp_fetch_probe_grant_slot] &&
      !comp_fetch_deep_stage[comp_fetch_probe_grant_slot] &&
      comp_word_start_event[comp_fetch_probe_grant_slot]));
wire comp_fetch_grant_launch = comp_fetch_grant_valid &&
    !comp_fetch_grant_deadline_miss && !gfx_req_pending &&
    !gfx_scroll_req_blocked &&
    comp_fetch_pending_eff[comp_fetch_grant_slot];
wire comp_fetch_probe_grant_launch = comp_fetch_probe_grant_valid &&
    !comp_fetch_probe_grant_deadline_miss && !gfx_req_probe_pending &&
    !gfx_scroll_req_blocked && !gp_rom_probe_mode &&
    comp_fetch_pending_eff[comp_fetch_probe_grant_slot];
wire [15:0] comp_attr_eff0 = gp_attr_eff_calc(comp_tile_attr[0], comp_tile_code[0], gp_live_tile_word_mode);
wire [15:0] comp_attr_eff1 = gp_attr_eff_calc(comp_tile_attr[1], comp_tile_code[1], gp_live_tile_word_mode);
wire [15:0] comp_attr_eff2 = gp_attr_eff_calc(comp_tile_attr[2], comp_tile_code[2], gp_live_tile_word_mode);
wire [15:0] comp_attr_eff3 = gp_attr_eff_calc(comp_tile_attr[3], comp_tile_code[3], gp_live_tile_word_mode);
wire [15:0] comp_attr_eff4 = gp_attr_eff_calc(comp_tile_attr[4], comp_tile_code[4], gp_live_tile_word_mode);
wire [15:0] comp_attr_eff5 = gp_attr_eff_calc(comp_tile_attr[5], comp_tile_code[5], gp_live_tile_word_mode);
wire [3:0] comp_pri0 = comp_attr_eff0[11:8] & 4'he;
wire [3:0] comp_pri1 = comp_attr_eff1[11:8] & 4'he;
wire [3:0] comp_pri2 = comp_attr_eff2[11:8] & 4'he;
wire [3:0] comp_pri3 = comp_attr_eff3[11:8] & 4'he;
wire [3:0] comp_pri4 = comp_attr_eff4[11:8] & 4'he;
wire [3:0] comp_pri5 = comp_attr_eff5[11:8] & 4'he;
wire [15:0] comp_code_eff0 = gp_code_eff_calc(comp_tile_attr[0], comp_tile_code[0], gp_live_tile_word_mode);
wire [15:0] comp_code_eff1 = gp_code_eff_calc(comp_tile_attr[1], comp_tile_code[1], gp_live_tile_word_mode);
wire [15:0] comp_code_eff2 = gp_code_eff_calc(comp_tile_attr[2], comp_tile_code[2], gp_live_tile_word_mode);
wire [15:0] comp_code_eff3 = gp_code_eff_calc(comp_tile_attr[3], comp_tile_code[3], gp_live_tile_word_mode);
wire [15:0] comp_code_eff4 = gp_code_eff_calc(comp_tile_attr[4], comp_tile_code[4], gp_live_tile_word_mode);
wire [15:0] comp_code_eff5 = gp_code_eff_calc(comp_tile_attr[5], comp_tile_code[5], gp_live_tile_word_mode);
wire comp_nonzero0 = comp_code_eff0 != 16'h0000;
wire comp_nonzero1 = comp_code_eff1 != 16'h0000;
wire comp_nonzero2 = comp_code_eff2 != 16'h0000;
wire comp_nonzero3 = comp_code_eff3 != 16'h0000;
wire comp_nonzero4 = comp_code_eff4 != 16'h0000;
wire comp_nonzero5 = comp_code_eff5 != 16'h0000;
wire [AW-1:0] comp_addr_lo0 = gp_gfx_addr_calc(1'b0, comp_code_eff0, gp0_l0_word_in_tile, gp_live_gfx_map_mode, 1'b0);
wire [AW-1:0] comp_addr_hi0 = gp_gfx_addr_calc(1'b0, comp_code_eff0, gp0_l0_word_in_tile, gp_live_gfx_map_mode, 1'b1);
wire [AW-1:0] comp_addr_lo1 = gp_gfx_addr_calc(1'b0, comp_code_eff1, gp0_l1_word_in_tile, gp_live_gfx_map_mode, 1'b0);
wire [AW-1:0] comp_addr_hi1 = gp_gfx_addr_calc(1'b0, comp_code_eff1, gp0_l1_word_in_tile, gp_live_gfx_map_mode, 1'b1);
wire [AW-1:0] comp_addr_lo2 = gp_gfx_addr_calc(1'b0, comp_code_eff2, gp0_l2_word_in_tile, gp_live_gfx_map_mode, 1'b0);
wire [AW-1:0] comp_addr_hi2 = gp_gfx_addr_calc(1'b0, comp_code_eff2, gp0_l2_word_in_tile, gp_live_gfx_map_mode, 1'b1);
wire [AW-1:0] comp_addr_lo3 = gp_gfx_addr_calc(1'b1, comp_code_eff3, gp1_l0_word_in_tile, gp_live_gfx_map_mode, 1'b0);
wire [AW-1:0] comp_addr_hi3 = gp_gfx_addr_calc(1'b1, comp_code_eff3, gp1_l0_word_in_tile, gp_live_gfx_map_mode, 1'b1);
wire [AW-1:0] comp_addr_lo4 = gp_gfx_addr_calc(1'b1, comp_code_eff4, gp1_l1_word_in_tile, gp_live_gfx_map_mode, 1'b0);
wire [AW-1:0] comp_addr_hi4 = gp_gfx_addr_calc(1'b1, comp_code_eff4, gp1_l1_word_in_tile, gp_live_gfx_map_mode, 1'b1);
wire [AW-1:0] comp_addr_lo5 = gp_gfx_addr_calc(1'b1, comp_code_eff5, gp1_l2_word_in_tile, gp_live_gfx_map_mode, 1'b0);
wire [AW-1:0] comp_addr_hi5 = gp_gfx_addr_calc(1'b1, comp_code_eff5, gp1_l2_word_in_tile, gp_live_gfx_map_mode, 1'b1);

`ifdef BATSUGUN_HW_DEBUG
wire [5:0] video_profile_enqueue_mask = {
    (clkdiv == 4'd13) && !debug_frame_tick && gp1_l2_word_start &&
        comp_layer_fetch_en[5] && comp_nonzero5,
    (clkdiv == 4'd9) && gp1_l1_word_start &&
        comp_layer_fetch_en[4] && comp_nonzero4,
    (clkdiv == 4'd5) && gp1_l0_word_start &&
        comp_layer_fetch_en[3] && comp_nonzero3,
    (clkdiv == 4'd13) && !debug_frame_tick && gp0_l2_word_start &&
        comp_layer_fetch_en[2] && comp_nonzero2,
    (clkdiv == 4'd9) && gp0_l1_word_start &&
        comp_layer_fetch_en[1] && comp_nonzero1,
    (clkdiv == 4'd5) && gp0_l0_word_start &&
        comp_layer_fetch_en[0] && comp_nonzero0
};
wire video_profile_primary_launch = !gfx_req_deadline_miss &&
    comp_fetch_grant_launch;
wire [5:0] video_profile_primary_launch_mask =
    video_profile_primary_launch ?
        (6'b000001 << comp_fetch_grant_slot) : 6'b000000;
wire video_profile_probe_launch = !gfx_req_probe_deadline_miss &&
    comp_fetch_probe_grant_launch;
wire [5:0] video_profile_probe_launch_mask =
    video_profile_probe_launch ?
        (6'b000001 << comp_fetch_probe_grant_slot) : 6'b000000;
wire [5:0] video_profile_overwrite_mask = video_profile_enqueue_mask &
    comp_fetch_pending &
    ~(video_profile_primary_launch_mask | video_profile_probe_launch_mask);
wire video_profile_primary_complete = gfx_req_pending && gfx_req_phase &&
                                      gfx_scroll_slot_ok;
wire video_profile_probe_complete = gfx_req_probe_pending &&
                                    gfx_req_probe_phase && gfx_probe_slot_ok;
wire [4:0] gfx_req_cache_idx = {gfx_req_slot, gfx_req_target_x[4:3]};
wire [4:0] gfx_req_probe_cache_idx =
    {gfx_req_probe_slot, gfx_req_probe_target_x[4:3]};
wire video_profile_primary_stage_collision = video_profile_primary_complete &&
    comp_word_cache_ready[gfx_req_cache_idx] &&
    (comp_word_cache_target_x[gfx_req_cache_idx] == gfx_req_target_x) &&
    (comp_word_cache_target_y[gfx_req_cache_idx] == gfx_req_target_y);
wire video_profile_probe_stage_collision = video_profile_probe_complete &&
    comp_word_cache_ready[gfx_req_probe_cache_idx] &&
    (comp_word_cache_target_x[gfx_req_probe_cache_idx] == gfx_req_probe_target_x) &&
    (comp_word_cache_target_y[gfx_req_probe_cache_idx] == gfx_req_probe_target_y);
`endif

wire [3:0] comp_sample0 = gp_decode_sample_calc(
    GP_LIVE_PLANE_SWAP ? comp_latched_hi[0] : comp_latched_lo[0],
    GP_LIVE_PLANE_SWAP ? comp_latched_lo[0] : comp_latched_hi[0],
    gp0_l0_x_mod[2:0], gp_live_gfx_decode_mode);
wire [3:0] comp_sample1 = gp_decode_sample_calc(
    GP_LIVE_PLANE_SWAP ? comp_latched_hi[1] : comp_latched_lo[1],
    GP_LIVE_PLANE_SWAP ? comp_latched_lo[1] : comp_latched_hi[1],
    gp0_l1_x_mod[2:0], gp_live_gfx_decode_mode);
wire [3:0] comp_sample2 = gp_decode_sample_calc(
    GP_LIVE_PLANE_SWAP ? comp_latched_hi[2] : comp_latched_lo[2],
    GP_LIVE_PLANE_SWAP ? comp_latched_lo[2] : comp_latched_hi[2],
    gp0_l2_x_mod[2:0], gp_live_gfx_decode_mode);
wire [3:0] comp_sample3 = gp_decode_sample_calc(
    GP_LIVE_PLANE_SWAP ? comp_latched_hi[3] : comp_latched_lo[3],
    GP_LIVE_PLANE_SWAP ? comp_latched_lo[3] : comp_latched_hi[3],
    gp1_l0_x_mod[2:0], gp_live_gfx_decode_mode);
wire [3:0] comp_sample4 = gp_decode_sample_calc(
    GP_LIVE_PLANE_SWAP ? comp_latched_hi[4] : comp_latched_lo[4],
    GP_LIVE_PLANE_SWAP ? comp_latched_lo[4] : comp_latched_hi[4],
    gp1_l1_x_mod[2:0], gp_live_gfx_decode_mode);
wire [3:0] comp_sample5 = gp_decode_sample_calc(
    GP_LIVE_PLANE_SWAP ? comp_latched_hi[5] : comp_latched_lo[5],
    GP_LIVE_PLANE_SWAP ? comp_latched_lo[5] : comp_latched_hi[5],
    gp1_l2_x_mod[2:0], gp_live_gfx_decode_mode);
wire [14:0] comp_pixel0 = (gp_tile_area && comp_latched_valid[0] && (comp_sample0 != 4'h0)) ? {comp_latched_pri[0], comp_latched_color[0], comp_sample0} : 15'h0000;
wire [14:0] comp_pixel1 = (gp_tile_area && comp_latched_valid[1] && (comp_sample1 != 4'h0)) ? {comp_latched_pri[1], comp_latched_color[1], comp_sample1} : 15'h0000;
wire [14:0] comp_pixel2 = (gp_tile_area && comp_latched_valid[2] && (comp_sample2 != 4'h0)) ? {comp_latched_pri[2], comp_latched_color[2], comp_sample2} : 15'h0000;
wire [14:0] comp_pixel3 = (gp_tile_area && comp_latched_valid[3] && (comp_sample3 != 4'h0)) ? {comp_latched_pri[3], comp_latched_color[3], comp_sample3} : 15'h0000;
wire [14:0] comp_pixel4 = (gp_tile_area && comp_latched_valid[4] && (comp_sample4 != 4'h0)) ? {comp_latched_pri[4], comp_latched_color[4], comp_sample4} : 15'h0000;
wire [14:0] comp_pixel5 = (gp_tile_area && comp_latched_valid[5] && (comp_sample5 != 4'h0)) ? {comp_latched_pri[5], comp_latched_color[5], comp_sample5} : 15'h0000;
wire [14:0] gp0_comp_pixel = gp_layer_priority_mux(comp_pixel0, comp_pixel1, comp_pixel2);
wire [14:0] gp1_comp_pixel = gp_layer_priority_mux(comp_pixel3, comp_pixel4, comp_pixel5);
wire [1:0] gp0_comp_src = gp_layer_priority_src(comp_pixel0, comp_pixel1, comp_pixel2);
wire [1:0] gp1_comp_src = gp_layer_priority_src(comp_pixel3, comp_pixel4, comp_pixel5);
wire gp0_obj_fill_opaque;
wire gp1_obj_fill_opaque;
wire legacy_obj_sprite_opaque = ENABLE_OBJ_SPRITE_COMPOSITE &&
                                !NO_OBJ_COMPOSITE_DIAG &&
                                (obj_line_sprite_sample != 4'h0);
wire [3:0] obj_line_sprite_pri_eff = OBJ_PRIORITY_EVEN_MASK_TEST ?
                                      (obj_line_sprite_pri & 4'he) :
                                      obj_line_sprite_pri;
wire [14:0] obj_sprite_pixel = {obj_line_sprite_pri_eff,
                                obj_line_sprite_color,
                                obj_line_sprite_sample};
wire gp0_legacy_obj_sprite_opaque = legacy_obj_sprite_opaque &&
                                    !obj_line_sprite_gp_sel;
wire gp1_legacy_obj_sprite_opaque = legacy_obj_sprite_opaque &&
                                     obj_line_sprite_gp_sel;
wire gp0_obj_sprite_opaque = ENABLE_OBJ_LINEBUFFER_COMPOSITE ?
                             gp0_obj_lb_opaque :
                             gp0_legacy_obj_sprite_opaque;
wire gp1_obj_sprite_opaque = ENABLE_OBJ_LINEBUFFER_COMPOSITE ?
                             gp1_obj_lb_opaque :
                             gp1_legacy_obj_sprite_opaque;
wire gp0_obj_opaque = gp0_obj_sprite_opaque || gp0_obj_fill_opaque;
wire gp1_obj_opaque = gp1_obj_sprite_opaque || gp1_obj_fill_opaque;
wire [14:0] gp0_obj_fill_live_pixel = ENABLE_OBJ_SILHOUETTE_COMPOSITE ?
                                      gp0_obj_fill_pixel : 15'h0000;
wire [14:0] gp1_obj_fill_live_pixel = ENABLE_OBJ_SILHOUETTE_COMPOSITE ?
                                      gp1_obj_fill_pixel : 15'h0000;
wire [14:0] gp0_obj_sprite_pixel = ENABLE_OBJ_LINEBUFFER_COMPOSITE ?
                                    gp0_obj_lb_pixel : obj_sprite_pixel;
wire [14:0] gp1_obj_sprite_pixel = ENABLE_OBJ_LINEBUFFER_COMPOSITE ?
                                    gp1_obj_lb_pixel : obj_sprite_pixel;
wire [14:0] gp0_obj_pixel = gp0_obj_sprite_opaque ? gp0_obj_sprite_pixel :
                                                     gp0_obj_fill_live_pixel;
wire [14:0] gp1_obj_pixel = gp1_obj_sprite_opaque ? gp1_obj_sprite_pixel :
                                                     gp1_obj_fill_live_pixel;
wire gp0_obj_over_tile = gp0_obj_opaque && (gp0_obj_pixel[14:11] >= gp0_comp_pixel[14:11]);
wire gp1_obj_over_tile = gp1_obj_opaque && (gp1_obj_pixel[14:11] >= gp1_comp_pixel[14:11]);
wire [14:0] gp0_vdp_pixel = gp0_obj_over_tile ? gp0_obj_pixel : gp0_comp_pixel;
wire [14:0] gp1_vdp_pixel = gp1_obj_over_tile ? gp1_obj_pixel : gp1_comp_pixel;
wire gp0_comp_opaque = gp0_vdp_pixel[10:0] != 11'h000;
wire gp1_comp_opaque = gp1_vdp_pixel[10:0] != 11'h000;
wire gp1_comp_wins = gp1_comp_opaque &&
                     (!gp0_comp_opaque || ((gp0_vdp_pixel[10:0] & 11'h780) > (gp1_vdp_pixel[10:0] & 11'h780)));
wire [14:0] gp_all_pixel = gp1_comp_wins ? gp1_vdp_pixel : gp0_vdp_pixel;
wire [3:0] gp0_src_code = gp0_obj_over_tile ? 4'd6 :
                           (gp0_comp_src == 2'd0) ? 4'd0 :
                           (gp0_comp_src == 2'd1) ? 4'd1 :
                           (gp0_comp_src == 2'd2) ? 4'd2 : 4'd8;
wire [3:0] gp1_src_code = gp1_obj_over_tile ? 4'd7 :
                           (gp1_comp_src == 2'd0) ? 4'd3 :
                           (gp1_comp_src == 2'd1) ? 4'd4 :
                           (gp1_comp_src == 2'd2) ? 4'd5 : 4'd8;
wire [3:0] gp_edge_src_code = gp1_comp_wins ? gp1_src_code : gp0_src_code;
wire gp_edge_src_opaque = gp_tile_area && (gp_all_pixel[10:0] != 11'h000);
wire [14:0] gp_debug_pixel = gp_render_sel ?
                              ((gp_render_layer == 2'd0) ? comp_pixel3 :
                               (gp_render_layer == 2'd1) ? comp_pixel4 : comp_pixel5) :
                              ((gp_render_layer == 2'd0) ? comp_pixel0 :
                               (gp_render_layer == 2'd1) ? comp_pixel1 : comp_pixel2);
wire [15:0] gp_debug_attr_eff = gp_render_sel ?
                                ((gp_render_layer == 2'd0) ? comp_attr_eff3 :
                                 (gp_render_layer == 2'd1) ? comp_attr_eff4 : comp_attr_eff5) :
                                ((gp_render_layer == 2'd0) ? comp_attr_eff0 :
                                 (gp_render_layer == 2'd1) ? comp_attr_eff1 : comp_attr_eff2);
wire [15:0] gp_debug_code_eff = gp_render_sel ?
                                ((gp_render_layer == 2'd0) ? comp_code_eff3 :
                                 (gp_render_layer == 2'd1) ? comp_code_eff4 : comp_code_eff5) :
                                ((gp_render_layer == 2'd0) ? comp_code_eff0 :
                                 (gp_render_layer == 2'd1) ? comp_code_eff1 : comp_code_eff2);
wire [14:0] gp_display_pixel = gp_composite_mode ? gp_all_pixel : gp_debug_pixel;
wire [10:0] gp_pixel_lut_addr = gp_display_pixel[10:0];
wire [3:0] gp_pixel_priority = gp_display_pixel[14:11];
wire gp_pixel_opaque = !gp_tile_hex_mode && gp_tile_area && (gp_pixel_lut_addr != 11'h000);
reg  [11:0] pal_scan_addr = 12'h000;
wire [15:0] pal_scan_q;
wire [7:0] gp_pal_red = {pal_scan_q[4:0], pal_scan_q[4:2]};
wire [7:0] gp_pal_green = {pal_scan_q[9:5], pal_scan_q[9:7]};
wire [7:0] gp_pal_blue = {pal_scan_q[14:10], pal_scan_q[14:12]};
always @* begin
    gp0_obj_box_bit = 1'b0;
    gp1_obj_box_bit = 1'b0;
    gp0_obj_fill_bit = 1'b0;
    gp1_obj_fill_bit = 1'b0;
    gp0_obj_sprite_sample = 4'h0;
    gp1_obj_sprite_sample = 4'h0;
    gp0_obj_sprite_raw_bit = 1'b0;
    gp1_obj_sprite_raw_bit = 1'b0;
    gp0_obj_sprite_pri = 4'h0;
    gp1_obj_sprite_pri = 4'h0;
    gp0_obj_fill_pixel = 15'h0000;
    gp1_obj_fill_pixel = 15'h0000;
    obj_box_dx = 9'd0;
    obj_box_dy = 9'd0;
    obj_sprite_x = 3'd0;
    obj_sprite_y = 3'd0;
    obj_box_idx = 5'd0;
    obj_line_fetch_x = hcnt[8:0] + 9'd8;
    obj_line_dx = 9'd0;
    obj_line_dy = 9'd0;
    obj_line_w_tiles = 5'd0;
    obj_line_h_tiles = 5'd0;
    obj_line_tile_x = 5'd0;
    obj_line_tile_y = 5'd0;
    obj_line_tile_index = 9'd0;
    obj_line_pick_valid = 1'b0;
    obj_line_pick_gp_sel = 1'b0;
    obj_line_pick_code = 18'h00000;
    obj_line_pick_row = 3'd0;
    obj_line_pick_color = 7'h00;
    obj_line_pick_pri = 4'h0;
    obj_line_pick_flipx = 1'b0;

    for (obj_box_i = 0; obj_box_i < OBJ_DEBUG_COUNT; obj_box_i = obj_box_i + 1) begin
        obj_box_idx = obj_box_i[4:0];
        if (obj_box_i < gp0_obj_count) begin
            obj_box_dx = hcnt[8:0] - gp0_obj_x[obj_box_i];
            obj_box_dy = vcnt - gp0_obj_y[obj_box_i];
            if ((obj_box_dx < {1'b0, gp0_obj_w[obj_box_i]}) &&
                (obj_box_dy < {1'b0, gp0_obj_h[obj_box_i]})) begin
                gp0_obj_fill_bit = 1'b1;
                if ((obj_box_dx == 9'd0) || (obj_box_dy == 9'd0) ||
                    (obj_box_dx == ({1'b0, gp0_obj_w[obj_box_i]} - 9'd1)) ||
                    (obj_box_dy == ({1'b0, gp0_obj_h[obj_box_i]} - 9'd1))) begin
                    gp0_obj_box_bit = 1'b1;
                end
                if (gp0_obj_attr[obj_box_i][11:8] >= gp0_obj_fill_pixel[14:11]) begin
                    gp0_obj_fill_pixel = gp_obj_silhouette_pixel(gp0_obj_attr[obj_box_i]);
                end
            end
        end

        if (debug_object_line_en && (obj_box_i < gp0_obj_count)) begin
            obj_line_dx = obj_line_fetch_x - gp0_obj_x[obj_box_i];
            obj_line_dy = vcnt - gp0_obj_y[obj_box_i];
            if ((obj_line_dx < {1'b0, gp0_obj_w[obj_box_i]}) &&
                (obj_line_dy < {1'b0, gp0_obj_h[obj_box_i]})) begin
                obj_line_w_tiles = gp0_obj_w[obj_box_i][7:3];
                obj_line_h_tiles = gp0_obj_h[obj_box_i][7:3];
                obj_line_tile_x = gp0_obj_attr[obj_box_i][12] ?
                                  (obj_line_w_tiles - 5'd1 - {1'b0, obj_line_dx[6:3]}) :
                                  {1'b0, obj_line_dx[6:3]};
                obj_line_tile_y = gp0_obj_attr[obj_box_i][13] ?
                                  (obj_line_h_tiles - 5'd1 - {1'b0, obj_line_dy[6:3]}) :
                                  {1'b0, obj_line_dy[6:3]};
                obj_line_tile_index = gp_obj_subtile_index_calc(obj_line_w_tiles,
                                                                obj_line_tile_x,
                                                                obj_line_tile_y);
                if ((!obj_line_pick_valid) ||
                    (gp0_obj_attr[obj_box_i][11:8] >= obj_line_pick_pri)) begin
                    obj_line_pick_valid = 1'b1;
                    obj_line_pick_gp_sel = 1'b0;
                    obj_line_pick_code = gp0_obj_code[obj_box_i] +
                                         {9'd0, obj_line_tile_index};
                    obj_line_pick_row = gp0_obj_attr[obj_box_i][13] ?
                                        (3'd7 - obj_line_dy[2:0]) :
                                        obj_line_dy[2:0];
                    obj_line_pick_color = gp0_obj_attr[obj_box_i][7:2];
                    obj_line_pick_pri = gp0_obj_attr[obj_box_i][11:8];
                    obj_line_pick_flipx = gp0_obj_attr[obj_box_i][12];
                end
            end
        end

        if (obj_box_i < gp1_obj_count) begin
            obj_box_dx = hcnt[8:0] - gp1_obj_x[obj_box_i];
            obj_box_dy = vcnt - gp1_obj_y[obj_box_i];
            if ((obj_box_dx < {1'b0, gp1_obj_w[obj_box_i]}) &&
                (obj_box_dy < {1'b0, gp1_obj_h[obj_box_i]})) begin
                gp1_obj_fill_bit = 1'b1;
                if ((obj_box_dx == 9'd0) || (obj_box_dy == 9'd0) ||
                    (obj_box_dx == ({1'b0, gp1_obj_w[obj_box_i]} - 9'd1)) ||
                    (obj_box_dy == ({1'b0, gp1_obj_h[obj_box_i]} - 9'd1))) begin
                    gp1_obj_box_bit = 1'b1;
                end
                if (gp1_obj_attr[obj_box_i][11:8] >= gp1_obj_fill_pixel[14:11]) begin
                    gp1_obj_fill_pixel = gp_obj_silhouette_pixel(gp1_obj_attr[obj_box_i]);
                end
            end
        end

        if (debug_object_line_en && (obj_box_i < gp1_obj_count)) begin
            obj_line_dx = obj_line_fetch_x - gp1_obj_x[obj_box_i];
            obj_line_dy = vcnt - gp1_obj_y[obj_box_i];
            if ((obj_line_dx < {1'b0, gp1_obj_w[obj_box_i]}) &&
                (obj_line_dy < {1'b0, gp1_obj_h[obj_box_i]})) begin
                obj_line_w_tiles = gp1_obj_w[obj_box_i][7:3];
                obj_line_h_tiles = gp1_obj_h[obj_box_i][7:3];
                obj_line_tile_x = gp1_obj_attr[obj_box_i][12] ?
                                  (obj_line_w_tiles - 5'd1 - {1'b0, obj_line_dx[6:3]}) :
                                  {1'b0, obj_line_dx[6:3]};
                obj_line_tile_y = gp1_obj_attr[obj_box_i][13] ?
                                  (obj_line_h_tiles - 5'd1 - {1'b0, obj_line_dy[6:3]}) :
                                  {1'b0, obj_line_dy[6:3]};
                obj_line_tile_index = gp_obj_subtile_index_calc(obj_line_w_tiles,
                                                                obj_line_tile_x,
                                                                obj_line_tile_y);
                if ((!obj_line_pick_valid) ||
                    (gp1_obj_attr[obj_box_i][11:8] >= obj_line_pick_pri)) begin
                    obj_line_pick_valid = 1'b1;
                    obj_line_pick_gp_sel = 1'b1;
                    obj_line_pick_code = gp1_obj_code[obj_box_i] +
                                         {9'd0, obj_line_tile_index};
                    obj_line_pick_row = gp1_obj_attr[obj_box_i][13] ?
                                        (3'd7 - obj_line_dy[2:0]) :
                                        obj_line_dy[2:0];
                    obj_line_pick_color = gp1_obj_attr[obj_box_i][7:2];
                    obj_line_pick_pri = gp1_obj_attr[obj_box_i][11:8];
                    obj_line_pick_flipx = gp1_obj_attr[obj_box_i][12];
                end
            end
        end
    end
end
assign gp0_obj_fill_opaque = ENABLE_OBJ_SILHOUETTE_COMPOSITE && gp0_obj_fill_bit;
assign gp1_obj_fill_opaque = ENABLE_OBJ_SILHOUETTE_COMPOSITE && gp1_obj_fill_bit;
wire debug_object_box_bit = debug_object_box_en && gp_tile_area &&
                            (gp0_obj_box_bit || gp1_obj_box_bit);
wire debug_object_sprite_bit = debug_object_line_en && gp_tile_area &&
                               (obj_line_sprite_sample != 4'h0);
wire debug_object_raw_bit = debug_object_line_en && gp_tile_area &&
                            obj_line_sprite_raw_bit;
wire debug_object_fill_bit = debug_object_fill_en && gp_tile_area &&
                             (hcnt[1] ^ vcnt[1]) &&
                             (gp0_obj_fill_bit || gp1_obj_fill_bit);
wire debug_object_status_area = debug_object_fill_en && gp_tile_area &&
                                (vcnt < 9'd8) && (hcnt < 10'd96);
wire [3:0] debug_object_status_col = hcnt[6:3];
wire debug_object_tile_area = debug_object_fill_en && gp_tile_area &&
                              (hcnt >= 10'd112) && (hcnt < 10'd240) &&
                              (vcnt < 9'd32);
wire [9:0] debug_object_tile_x = hcnt - 10'd112;
wire [1:0] debug_object_tile_decode_mode = debug_object_tile_x[6:5];
wire [2:0] debug_object_tile_px = debug_object_tile_x[4:2];
wire [2:0] debug_object_tile_py = vcnt[4:2];
wire debug_object_tile_valid = obj_gfx_debug_slot_valid &&
                               obj_gfx_debug_valid[debug_object_tile_py];
wire [3:0] debug_object_tile_sample = debug_object_tile_valid ?
    gp_decode_probe_sample_calc(obj_gfx_debug_lo[debug_object_tile_py],
                                obj_gfx_debug_hi[debug_object_tile_py],
                                debug_object_tile_px, debug_object_tile_decode_mode) :
    4'h0;
wire debug_object_meta_area = debug_object_overlay_en && gp_tile_area &&
                              (hcnt >= 10'd8) && (hcnt < 10'd136) &&
                              (vcnt >= 9'd40) && (vcnt < 9'd168);
wire [9:0] debug_object_meta_x = hcnt - 10'd8;
wire [8:0] debug_object_meta_y = vcnt - 9'd40;
wire [2:0] debug_object_meta_row = debug_object_meta_y[6:4];
wire [1:0] debug_object_meta_digit = debug_object_meta_x[6:5];
reg  [15:0] debug_object_meta_word;
wire [3:0] debug_object_meta_nibble =
    (debug_object_meta_digit == 2'd0) ? debug_object_meta_word[15:12] :
    (debug_object_meta_digit == 2'd1) ? debug_object_meta_word[11:8] :
    (debug_object_meta_digit == 2'd2) ? debug_object_meta_word[7:4] :
                                        debug_object_meta_word[3:0];
wire [2:0] debug_object_meta_font_x = debug_object_meta_x[4:2];
wire [2:0] debug_object_meta_font_y = debug_object_meta_y[3:1];
wire [3:0] debug_object_meta_font_row =
    hex_glyph(debug_object_meta_nibble, debug_object_meta_font_y);
wire debug_object_meta_hex_bit = debug_object_meta_area &&
                                 (debug_object_meta_font_y < 3'd7) &&
                                 (debug_object_meta_font_x < 3'd4) &&
                                 debug_object_meta_font_row[3'd3 - debug_object_meta_font_x[1:0]];
always @* begin
    case (debug_object_meta_row)
        3'd0: debug_object_meta_word = obj_meta_counts;
        3'd1: debug_object_meta_word = obj_meta_attr;
        3'd2: debug_object_meta_word = obj_meta_code[15:0];
        3'd3: debug_object_meta_word = obj_meta_raw_x;
        3'd4: debug_object_meta_word = obj_meta_raw_y;
        3'd5: debug_object_meta_word = {obj_meta_w, obj_meta_h};
        3'd6: debug_object_meta_word = obj_gfx_debug_hit_valid ?
                                        obj_gfx_debug_hit_lo : 16'h0000;
        default: debug_object_meta_word = obj_gfx_debug_hit_valid ?
                                          obj_gfx_debug_hit_hi : 16'h0000;
    endcase
end
wire [15:0] debug_live0 = {gp_pixel_priority, gp_composite_mode, gp_tile_hex_mode, gp_tile_word_mode, gp_gfx_map_mode, gp_gfx_decode_mode, gp_render_sel, gp_render_layer, gp_pixel_opaque};
wire [15:0] debug_live1 = gp_debug_attr_eff;
wire [15:0] debug_live2 = gp_debug_code_eff;
wire [15:0] debug_live3 = gp_layer_scroll_x;
wire [15:0] debug_live4 = gp_layer_scroll_y;
wire [15:0] debug_live5 = gp_composite_mode ? gp0_comp_pixel : gp_debug_pixel;
wire [15:0] debug_live6 = gp_composite_mode ? gp1_comp_pixel : {1'b0, gp_debug_pixel};
wire [15:0] debug_live7 = pal_scan_q;
wire [15:0] gp_tile_hex_word = gp_tile_code_eff;
reg  [3:0] gp_tile_hex_nibble;
always @* begin
    case (gp_tile_x[4:3])
        2'd0: gp_tile_hex_nibble = gp_tile_hex_word[15:12];
        2'd1: gp_tile_hex_nibble = gp_tile_hex_word[11:8];
        2'd2: gp_tile_hex_nibble = gp_tile_hex_word[7:4];
        default: gp_tile_hex_nibble = gp_tile_hex_word[3:0];
    endcase
end
wire [3:0] gp_tile_hex_row = hex_glyph(gp_tile_hex_nibble, gp_tile_y[3:1]);
wire gp_tile_hex_bit = gp_tile_hex_code_mode && gp_tile_area && (gp_gfx_py < 4'd14) &&
                       gp_tile_hex_row[3'd3 - gp_tile_x[2:1]];
localparam [AW-1:0] GP_ROM_PROBE_ADDR0 = 22'h0000e7; // tp030_3l.bin +0x1ce / 2
localparam [AW-1:0] GP_ROM_PROBE_ADDR1 = 22'h080025; // tp030_3h.bin +0x04a / 2
localparam [AW-1:0] GP_ROM_PROBE_ADDR2 = 22'h1000e7; // tp030_4l.bin +0x1ce / 2
localparam [AW-1:0] GP_ROM_PROBE_ADDR3 = 22'h180025; // tp030_4h.bin +0x04a / 2
localparam [AW-1:0] GP_ROM_PROBE_ADDR4 = 22'h200105; // tp030_5.bin  +0x20a / 2
localparam [AW-1:0] GP_ROM_PROBE_ADDR5 = 22'h280105; // tp030_6.bin  +0x20a / 2
localparam [15:0] GP_ROM_PROBE_EXP0 = 16'h0080;
localparam [15:0] GP_ROM_PROBE_EXP1 = 16'h0101;
localparam [15:0] GP_ROM_PROBE_EXP2 = 16'h0080;
localparam [15:0] GP_ROM_PROBE_EXP3 = 16'h0001;
localparam [15:0] GP_ROM_PROBE_EXP4 = 16'hc6b5;
localparam [15:0] GP_ROM_PROBE_EXP5 = 16'h00f7;
reg  [AW-1:0] gfx_fixed_probe_next_addr;
always @* begin
    case (gfx_fixed_probe_idx)
        3'd0: gfx_fixed_probe_next_addr = GP_ROM_PROBE_ADDR0;
        3'd1: gfx_fixed_probe_next_addr = GP_ROM_PROBE_ADDR1;
        3'd2: gfx_fixed_probe_next_addr = GP_ROM_PROBE_ADDR2;
        3'd3: gfx_fixed_probe_next_addr = GP_ROM_PROBE_ADDR3;
        3'd4: gfx_fixed_probe_next_addr = GP_ROM_PROBE_ADDR4;
        default: gfx_fixed_probe_next_addr = GP_ROM_PROBE_ADDR5;
    endcase
end
wire gp_rom_probe_area = gp_rom_probe_mode && render_lhbl && render_lvbl &&
                         (hcnt >= 10'd32) && (hcnt < 10'd320) &&
                         (vcnt >= 9'd32) && (vcnt < 9'd128);
wire [9:0] gp_rom_probe_x = hcnt - 10'd32;
wire [8:0] gp_rom_probe_y = vcnt - 9'd32;
wire [2:0] gp_rom_probe_row = gp_rom_probe_y[6:4];
wire gp_rom_probe_row_valid = gp_rom_probe_area && (gp_rom_probe_row < 3'd6);
reg  [15:0] gp_rom_probe_actual_word;
reg  [15:0] gp_rom_probe_expected_word;
reg         gp_rom_probe_seen;
always @* begin
    case (gp_rom_probe_row)
        3'd0: begin
            gp_rom_probe_actual_word = gfx_fixed_probe_word0;
            gp_rom_probe_expected_word = GP_ROM_PROBE_EXP0;
            gp_rom_probe_seen = gfx_fixed_probe_seen[0];
        end
        3'd1: begin
            gp_rom_probe_actual_word = gfx_fixed_probe_word1;
            gp_rom_probe_expected_word = GP_ROM_PROBE_EXP1;
            gp_rom_probe_seen = gfx_fixed_probe_seen[1];
        end
        3'd2: begin
            gp_rom_probe_actual_word = gfx_fixed_probe_word2;
            gp_rom_probe_expected_word = GP_ROM_PROBE_EXP2;
            gp_rom_probe_seen = gfx_fixed_probe_seen[2];
        end
        3'd3: begin
            gp_rom_probe_actual_word = gfx_fixed_probe_word3;
            gp_rom_probe_expected_word = GP_ROM_PROBE_EXP3;
            gp_rom_probe_seen = gfx_fixed_probe_seen[3];
        end
        3'd4: begin
            gp_rom_probe_actual_word = gfx_fixed_probe_word4;
            gp_rom_probe_expected_word = GP_ROM_PROBE_EXP4;
            gp_rom_probe_seen = gfx_fixed_probe_seen[4];
        end
        default: begin
            gp_rom_probe_actual_word = gfx_fixed_probe_word5;
            gp_rom_probe_expected_word = GP_ROM_PROBE_EXP5;
            gp_rom_probe_seen = gfx_fixed_probe_seen[5];
        end
    endcase
end
wire gp_rom_probe_match = gp_rom_probe_seen && (gp_rom_probe_actual_word == gp_rom_probe_expected_word);
wire gp_rom_probe_actual_col = gp_rom_probe_row_valid && (gp_rom_probe_x < 10'd128);
wire gp_rom_probe_expected_col = gp_rom_probe_row_valid &&
                                 (gp_rom_probe_x >= 10'd144) && (gp_rom_probe_x < 10'd272);
wire [7:0] gp_rom_probe_hex_x = gp_rom_probe_expected_col ?
                                (gp_rom_probe_x - 10'd144) : gp_rom_probe_x[7:0];
wire [15:0] gp_rom_probe_display_word = gp_rom_probe_expected_col ?
                                        gp_rom_probe_expected_word : gp_rom_probe_actual_word;
wire [1:0] gp_rom_probe_digit = gp_rom_probe_hex_x[6:5];
wire [3:0] gp_rom_probe_nibble = (gp_rom_probe_digit == 2'd0) ? gp_rom_probe_display_word[15:12] :
                                  (gp_rom_probe_digit == 2'd1) ? gp_rom_probe_display_word[11:8] :
                                  (gp_rom_probe_digit == 2'd2) ? gp_rom_probe_display_word[7:4] :
                                                                 gp_rom_probe_display_word[3:0];
wire [2:0] gp_rom_probe_font_x = gp_rom_probe_hex_x[4:2];
wire [2:0] gp_rom_probe_font_y = gp_rom_probe_y[3:1];
wire [3:0] gp_rom_probe_font_row = hex_glyph(gp_rom_probe_nibble, gp_rom_probe_font_y);
wire gp_rom_probe_hex_bit = (gp_rom_probe_actual_col || gp_rom_probe_expected_col) &&
                            (gp_rom_probe_font_y < 3'd7) && (gp_rom_probe_font_x < 3'd4) &&
                            gp_rom_probe_font_row[3'd3 - gp_rom_probe_font_x[1:0]];
wire [8:0] obj_buffer_copy_line = (obj_debug_buffer_mode == 2'd1) ?
                                  (OBJ_CACHE_SCAN_V - 9'd1) :
                                  (V_END - 9'd1);
wire obj_buffer_copy_start_pre = (clkdiv == 4'd12) &&
                                 (hcnt == (H_TOTAL - 10'd1)) &&
                                 (vcnt == obj_buffer_copy_line);
wire obj_cache_scan_start_pre = (clkdiv == 4'd12) &&
                                (hcnt == OBJ_CACHE_SCAN_H) &&
                                (vcnt == OBJ_CACHE_SCAN_V);
reg obj_buffer_copy_start = 1'b0;
reg obj_cache_scan_start_tick = 1'b0;

// Decode one clock early, then fan out a registered pulse on the original
// clkdiv==13 edge. This removes raw raster counters from thousands of enables.
always @(posedge clk) begin
    if (cpu_reset_base) begin
        obj_buffer_copy_start <= 1'b0;
        obj_cache_scan_start_tick <= 1'b0;
    end else begin
        obj_buffer_copy_start <= obj_buffer_copy_start_pre;
        obj_cache_scan_start_tick <= obj_cache_scan_start_pre;
    end
end
wire obj_scan_live_enable = (obj_debug_buffer_mode == 2'd2) &&
                             obj_cache_scan_active;
wire gfx_aux_req_pending = obj_lb_req_pending || obj_line_req_pending ||
                           obj_gfx_req_pending || gp_rom_probe_mode;
wire [8:0] obj_line_req_x_deadline = obj_line_req_target_x - hcnt[8:0];
wire obj_line_req_start_urgent = obj_line_req_pending &&
                                 (obj_line_req_phase == 3'd0) &&
                                 (obj_line_req_target_y == vcnt) &&
                                 (obj_line_req_x_deadline <= OBJ_REQ_URGENT_PIXELS);
wire obj_lb_tile_quiet = !gfx_req_pending && !gfx_req_probe_pending &&
                         !(|comp_fetch_pending_eff) &&
                         !comp_fetch_grant_valid &&
                         !comp_fetch_probe_grant_valid;
wire obj_lb_uncontended_hblank = (hcnt >= H_END) &&
                                   (hcnt < H_PREFETCH_START);
wire obj_lb_queue_urgent = obj_lb_frame_active && (vcnt < V_END) &&
                           (obj_lb_target_y > vcnt) &&
                           (obj_lb_target_ahead <= OBJ_LB_URGENT_LINES);
wire obj_lb_slot_start_safe = obj_lb_uncontended_hblank ||
                              obj_lb_tile_quiet || obj_lb_queue_urgent;
wire obj_lb_slot_grant = obj_linebuffer_render_en && obj_lb_req_pending &&
                         (obj_lb_req_hold || obj_lb_slot_start_safe) &&
                         !gp_rom_probe_mode;
wire obj_line_slot_grant = !obj_lb_req_pending && obj_line_render_en &&
                           obj_line_req_pending &&
                           !obj_line_primary_commit_pending &&
                           !obj_line_alt_commit_pending &&
                           !gp_rom_probe_mode;
wire obj_gfx_slot_grant = debug_object_cache_en && obj_gfx_req_pending &&
                          !obj_lb_req_pending && !obj_line_req_pending &&
                          !gp_rom_probe_mode;
wire obj_legacy_prefetch_miss_event = (clkdiv == 4'd0) &&
                                      obj_line_word_start &&
                                      (gp0_obj_fill_bit || gp1_obj_fill_bit) &&
                                      !obj_line_prefetch_match &&
                                      !obj_line_prefetch1_match &&
                                      !obj_line_prefetch2_match &&
                                      !obj_line_extra_match;
wire obj_prefetch_miss_event = obj_linebuffer_render_en ?
                               obj_lb_deadline_miss_event :
                               obj_legacy_prefetch_miss_event;
// Pressure row 0F: an object-covered word missed while its matching graphics
// request was still in flight. This separates fetch latency from scan/pick
// readiness behind the broader row 0E miss counter.
wire obj_urgent_wait_event = obj_linebuffer_render_en ?
    (obj_lb_deadline_miss_event && obj_lb_req_pending) :
    (obj_prefetch_miss_event && obj_line_req_pending &&
     (obj_line_req_target_y == vcnt) &&
     (obj_line_req_target_x == hcnt[8:0]));
wire [5:0] obj_miss_cause_mask = obj_linebuffer_render_en ?
    (obj_lb_deadline_miss_event ? {
        1'b0,
        1'b0,
        !obj_lb_req_pending,
        obj_lb_req_pending,
        1'b0,
        1'b1
    } : 6'b000000) :
    (obj_prefetch_miss_event ? {
        obj_seq_pick_launchable,
        obj_seq_pick_ready && !obj_seq_pick_valid,
        obj_seq_scan_active || obj_seq_scan_done_pending,
        obj_line_req_pending,
        !obj_line_prefetch_space,
        1'b1
    } : 6'b000000);

always @(posedge clk) begin
    if (video_runtime_reset) begin
        obj_miss_cause_count_event <= 6'b000000;
        obj_prefetch_miss_count_event <= 1'b0;
        obj_urgent_wait_count_event <= 1'b0;
    end else begin
        obj_miss_cause_count_event <= obj_miss_cause_mask;
        obj_prefetch_miss_count_event <= obj_prefetch_miss_event;
        obj_urgent_wait_count_event <= obj_urgent_wait_event;
    end
end

always @(posedge clk) begin
    if (video_runtime_reset) begin
        for (pressure_i = 0; pressure_i < 6; pressure_i = pressure_i + 1) begin
            comp_miss_count[pressure_i] <= 8'h00;
            comp_miss_latched[pressure_i] <= 8'h00;
            comp_miss_peak[pressure_i] <= 8'h00;
            comp_miss_display[pressure_i] <= 8'h00;
            obj_miss_cause_count[pressure_i] <= 8'h00;
            obj_miss_cause_latched[pressure_i] <= 8'h00;
            obj_miss_cause_peak[pressure_i] <= 8'h00;
            obj_miss_cause_display[pressure_i] <= 8'h00;
        end
        comp_miss_line_seen <= 6'b000000;
        obj_miss_cause_line_seen <= 6'b000000;
        obj_prefetch_miss_count <= 8'h00;
        obj_prefetch_miss_latched <= 8'h00;
        obj_prefetch_miss_peak <= 8'h00;
        obj_prefetch_miss_display <= 8'h00;
        obj_prefetch_miss_line_seen <= 1'b0;
        obj_urgent_wait_count <= 8'h00;
        obj_urgent_wait_latched <= 8'h00;
        obj_urgent_wait_peak <= 8'h00;
        obj_urgent_wait_display <= 8'h00;
        obj_urgent_wait_line_seen <= 1'b0;
        pressure_hold_count <= 7'd0;
        gp0_obj_live_peak <= 8'h00;
        gp1_obj_live_peak <= 8'h00;
        gp0_obj_visible_peak <= 8'h00;
        gp1_obj_visible_peak <= 8'h00;
        gp0_obj_cached_peak <= 8'h00;
        gp1_obj_cached_peak <= 8'h00;
        gp0_obj_visible_overflow_peak <= 8'h00;
        gp1_obj_visible_overflow_peak <= 8'h00;
        gp0_obj_live_display <= 8'h00;
        gp1_obj_live_display <= 8'h00;
        gp0_obj_visible_display <= 8'h00;
        gp1_obj_visible_display <= 8'h00;
        gp0_obj_cached_display <= 8'h00;
        gp1_obj_cached_display <= 8'h00;
        gp0_obj_visible_overflow_display <= 8'h00;
        gp1_obj_visible_overflow_display <= 8'h00;
    end else if (debug_frame_tick) begin
        for (pressure_i = 0; pressure_i < 6; pressure_i = pressure_i + 1) begin
            comp_miss_latched[pressure_i] <= comp_miss_count[pressure_i];
            obj_miss_cause_latched[pressure_i] <= obj_miss_cause_count[pressure_i];
            if (pressure_hold_count == PRESSURE_HOLD_FRAMES) begin
                comp_miss_display[pressure_i] <= max8(comp_miss_peak[pressure_i],
                                                      comp_miss_count[pressure_i]);
                comp_miss_peak[pressure_i] <= 8'h00;
                obj_miss_cause_display[pressure_i] <= max8(obj_miss_cause_peak[pressure_i],
                                                           obj_miss_cause_count[pressure_i]);
                obj_miss_cause_peak[pressure_i] <= 8'h00;
            end else begin
                comp_miss_peak[pressure_i] <= max8(comp_miss_peak[pressure_i],
                                                   comp_miss_count[pressure_i]);
                obj_miss_cause_peak[pressure_i] <= max8(obj_miss_cause_peak[pressure_i],
                                                        obj_miss_cause_count[pressure_i]);
            end
            comp_miss_count[pressure_i] <= 8'h00;
            obj_miss_cause_count[pressure_i] <= 8'h00;
        end
        comp_miss_line_seen <= 6'b000000;
        obj_miss_cause_line_seen <= 6'b000000;
        obj_prefetch_miss_latched <= obj_prefetch_miss_count;
        if (pressure_hold_count == PRESSURE_HOLD_FRAMES) begin
            obj_prefetch_miss_display <= max8(obj_prefetch_miss_peak,
                                              obj_prefetch_miss_count);
            obj_prefetch_miss_peak <= 8'h00;
            obj_urgent_wait_display <= max8(obj_urgent_wait_peak,
                                            obj_urgent_wait_count);
            obj_urgent_wait_peak <= 8'h00;
            gp0_obj_live_display <= max8(gp0_obj_live_peak,
                                         gp0_obj_live_latched);
            gp0_obj_live_peak <= 8'h00;
            gp1_obj_live_display <= max8(gp1_obj_live_peak,
                                         gp1_obj_live_latched);
            gp1_obj_live_peak <= 8'h00;
            gp0_obj_visible_display <= max8(gp0_obj_visible_peak,
                                            gp0_obj_visible_latched);
            gp0_obj_visible_peak <= 8'h00;
            gp1_obj_visible_display <= max8(gp1_obj_visible_peak,
                                            gp1_obj_visible_latched);
            gp1_obj_visible_peak <= 8'h00;
            gp0_obj_cached_display <= max8(gp0_obj_cached_peak,
                                           gp0_obj_cached_latched);
            gp0_obj_cached_peak <= 8'h00;
            gp1_obj_cached_display <= max8(gp1_obj_cached_peak,
                                           gp1_obj_cached_latched);
            gp1_obj_cached_peak <= 8'h00;
            gp0_obj_visible_overflow_display <= max8(gp0_obj_visible_overflow_peak,
                                                     gp0_obj_visible_overflow_latched);
            gp0_obj_visible_overflow_peak <= 8'h00;
            gp1_obj_visible_overflow_display <= max8(gp1_obj_visible_overflow_peak,
                                                     gp1_obj_visible_overflow_latched);
            gp1_obj_visible_overflow_peak <= 8'h00;
            pressure_hold_count <= 7'd0;
        end else begin
            obj_prefetch_miss_peak <= max8(obj_prefetch_miss_peak,
                                           obj_prefetch_miss_count);
            obj_urgent_wait_peak <= max8(obj_urgent_wait_peak,
                                         obj_urgent_wait_count);
            gp0_obj_live_peak <= max8(gp0_obj_live_peak,
                                      gp0_obj_live_latched);
            gp1_obj_live_peak <= max8(gp1_obj_live_peak,
                                      gp1_obj_live_latched);
            gp0_obj_visible_peak <= max8(gp0_obj_visible_peak,
                                         gp0_obj_visible_latched);
            gp1_obj_visible_peak <= max8(gp1_obj_visible_peak,
                                         gp1_obj_visible_latched);
            gp0_obj_cached_peak <= max8(gp0_obj_cached_peak,
                                        gp0_obj_cached_latched);
            gp1_obj_cached_peak <= max8(gp1_obj_cached_peak,
                                        gp1_obj_cached_latched);
            gp0_obj_visible_overflow_peak <= max8(gp0_obj_visible_overflow_peak,
                                                  gp0_obj_visible_overflow_latched);
            gp1_obj_visible_overflow_peak <= max8(gp1_obj_visible_overflow_peak,
                                                  gp1_obj_visible_overflow_latched);
            pressure_hold_count <= pressure_hold_count + 7'd1;
        end
        obj_prefetch_miss_count <= 8'h00;
        obj_prefetch_miss_line_seen <= 1'b0;
        obj_urgent_wait_latched <= obj_urgent_wait_count;
        obj_urgent_wait_count <= 8'h00;
        obj_urgent_wait_line_seen <= 1'b0;
    end else if (debug_line_tick) begin
        comp_miss_line_seen <= 6'b000000;
        obj_miss_cause_line_seen <= 6'b000000;
        obj_prefetch_miss_line_seen <= 1'b0;
        obj_urgent_wait_line_seen <= 1'b0;
    end else begin
        for (pressure_i = 0; pressure_i < 6; pressure_i = pressure_i + 1) begin
            if (comp_word_miss_mask[pressure_i] &&
                render_lhbl && render_lvbl &&
                !comp_miss_line_seen[pressure_i] &&
                (comp_miss_count[pressure_i] != 8'hff)) begin
                comp_miss_count[pressure_i] <= comp_miss_count[pressure_i] + 8'h01;
                comp_miss_line_seen[pressure_i] <= 1'b1;
            end
            if (obj_miss_cause_count_event[pressure_i] &&
                !obj_miss_cause_line_seen[pressure_i] &&
                (obj_miss_cause_count[pressure_i] != 8'hff)) begin
                obj_miss_cause_count[pressure_i] <= obj_miss_cause_count[pressure_i] + 8'h01;
                obj_miss_cause_line_seen[pressure_i] <= 1'b1;
            end
        end
        if (obj_prefetch_miss_count_event &&
            !obj_prefetch_miss_line_seen &&
            (obj_prefetch_miss_count != 8'hff)) begin
            obj_prefetch_miss_count <= obj_prefetch_miss_count + 8'h01;
            obj_prefetch_miss_line_seen <= 1'b1;
        end
        if (obj_urgent_wait_count_event &&
            !obj_urgent_wait_line_seen &&
            (obj_urgent_wait_count != 8'hff)) begin
            obj_urgent_wait_count <= obj_urgent_wait_count + 8'h01;
            obj_urgent_wait_line_seen <= 1'b1;
        end
    end
end

assign rom_slot_addr = rom_probe_done ? cpu_addr[18:1] : {16'd0, rom_probe_addr};
assign rom_slot_cs = rom_probe_done ? cpu_rom_cs : rom_probe_cs;
assign gfx_scroll_slot_addr = gfx_req_phase ? gfx_req_high_addr : gfx_req_addr;
assign gfx_scroll_slot_cs = gfx_req_pending && !video_profile_mem_probe;
assign gfx_obj_slot_addr = obj_lb_slot_grant ? obj_lb_req_addr :
                           (obj_line_slot_grant ? obj_line_req_addr :
                                                  obj_gfx_req_addr);
assign gfx_obj_slot_cs = (obj_lb_slot_grant || obj_line_slot_grant ||
                          obj_gfx_slot_grant) &&
                         !video_profile_mem_probe;
assign gfx_probe_slot_addr = video_profile_mem_probe ? video_profile_mem_addr :
                             (gp_rom_probe_mode ? gfx_fixed_probe_addr :
                              (gfx_req_probe_phase ? gfx_req_probe_high_addr :
                                                     gfx_req_probe_addr));
assign gfx_probe_slot_cs = video_profile_mem_probe ? 1'b1 :
                           (gp_rom_probe_mode ? gfx_fixed_probe_cs :
                                                gfx_req_probe_pending);
wire [15:0] diag_flags = {
    ever_ioctl_rom,
    ioctl_rom,
    ever_ioctl_wr,
    ioctl_wr,
    ever_dwnld_busy,
    dwnld_busy,
    ever_prog_we,
    prog_we,
    ever_prog_ack,
    prog_ack,
    ever_rom_probe_rd,
    rom_slot_rd,
    ever_rom_probe_ok,
    rom_slot_ok,
    prog_rdy,
    prog_dok
};
// Keep renderer timing combinational; exported blanks are latched with RGB.
assign render_lhbl = (hcnt >= H_START) && (hcnt < H_END);
assign render_lvbl = (vcnt >= V_START) && (vcnt < V_END);
assign ioctl_din = 8'h00;

assign ba1_addr = gfx_sdram_addr;
assign ba2_addr = oki_sdram_addr;
assign ba3_addr = {AW{1'b0}};
assign ba_rd = {1'b0, oki_rom_rd, gfx_slot_rd, rom_slot_rd};
assign ba_wr = 4'b0000;
assign ba0_din = 16'h0000;
assign ba1_din = 16'h0000;
assign ba2_din = 16'h0000;
assign ba3_din = 16'h0000;
assign ba0_dsn = 2'b11;
assign ba1_dsn = 2'b11;
assign ba2_dsn = 2'b11;
assign ba3_dsn = 2'b11;

assign sample = sound_sample;
assign snd_left = sound_mono;
assign snd_right = sound_mono;
assign snd_vu = 6'b000000;
assign snd_peak = 1'b0;

assign debug_bus = rom_probe_match ? cpu_last_addr[15:8] : {coin[0], cab_1p[0], joystick1[5:0]};
assign debug_view = rom_probe_match ? cpu_diag_flags[15:8] : diag_flags[15:8];

wire download_prom_we;
wire [AW-1:0] dwnld_prog_addr;
wire [15:0] dwnld_prog_data;
wire [1:0] dwnld_prog_mask;
wire [1:0] dwnld_prog_ba;
wire dwnld_prog_rd;
wire dwnld_prog_we;
wire dwnld_gp0_gfx = (dwnld_prog_ba == 2'd1) &&
                     (dwnld_prog_addr < 22'h200000);
wire dwnld_gp1_gfx = (dwnld_prog_ba == 2'd1) &&
                     (dwnld_prog_addr >= 22'h200000) &&
                     (dwnld_prog_addr < 22'h300000);
wire [AW-1:0] dwnld_gp0_packed_addr =
    {{(AW-21){1'b0}}, dwnld_prog_addr[19:0], dwnld_prog_addr[20]};
wire [AW-1:0] dwnld_gp1_packed_addr = 22'h200000 |
    {{(AW-20){1'b0}}, dwnld_prog_addr[18:0], dwnld_prog_addr[19]};

assign prog_addr = dwnld_gp0_gfx ? dwnld_gp0_packed_addr :
                   (dwnld_gp1_gfx ? dwnld_gp1_packed_addr :
                                    dwnld_prog_addr);
assign prog_data = dwnld_prog_data;
assign prog_mask = dwnld_prog_mask;
assign prog_ba = dwnld_prog_ba;
assign prog_rd = dwnld_prog_rd;
assign prog_we = dwnld_prog_we;

assign dwnld_busy = ioctl_rom | prog_we | download_prom_we;

// Releasing a ROM requester while the SDRAM controller is still leaving its
// programming state can strand its first transaction. Keep video requesters
// idle for about 11 ms after each ROM download before starting normal reads.
always @(posedge clk) begin
    if (rst || dwnld_busy || ioctl_rom)
        gfx_startup_hold <= 20'hfffff;
    else if (gfx_startup_reset)
        gfx_startup_hold <= gfx_startup_hold - 20'd1;
end

jtframe_dwnld #(
    .SDRAMW   ( AW+1       ),
    // The first 0x80000 bytes are 68K program ROM in bank 0.
    // Graphics use bank 1; the OKI sample ROM starts a separate bank 2.
    .BA1_START( 26'h080000 ),
    .BA2_START( 26'h680000 ),
    .SWAB     ( 1          )
) u_dwnld (
    .clk        ( clk               ),
    .ioctl_rom  ( ioctl_rom         ),
    .ioctl_addr ( ioctl_addr[25:0]  ),
    .ioctl_dout ( ioctl_dout        ),
    .ioctl_wr   ( ioctl_wr          ),
    .prog_addr  ( dwnld_prog_addr   ),
    .prog_data  ( dwnld_prog_data   ),
    .prog_mask  ( dwnld_prog_mask   ),
    .prog_we    ( dwnld_prog_we     ),
    .prog_rd    ( dwnld_prog_rd     ),
    .prog_ba    ( dwnld_prog_ba     ),
    .gfx4_en    ( 1'b0              ),
    .gfx8_en    ( 1'b0              ),
    .gfx16_en   ( 1'b0              ),
    .gfx16b_en  ( 1'b0              ),
    .gfx16c_en  ( 1'b0              ),
    .prom_we    ( download_prom_we  ),
    .header     (                   ),
    .sdram_ack  ( prog_ack          )
);

jtframe_rom_1slot #(
    .SDRAMW      ( AW      ),
    .SLOT0_DW    ( 16      ),
    .SLOT0_AW    ( 18      ),
    .SLOT0_LATCH ( 0       ),
    .SLOT0_OKLATCH( 0      ),
    .SLOT0_OFFSET( {AW{1'b0}} )
) u_rom_probe (
    // The 68000 must finish its current ROM fetch before it can acknowledge
    // the private save-state IRQ7. Renderer quiesce must not reset this path.
    .rst        ( rom_runtime_reset ),
    .clk        ( clk        ),
    .slot0_addr ( rom_slot_addr ),
    .slot0_dout ( rom_slot_dout ),
    .slot0_cs   ( rom_slot_cs   ),
    .slot0_ok   ( rom_slot_ok   ),
    .sdram_ack  ( ba_ack[0]  ),
    .sdram_rd   ( rom_slot_rd ),
    .sdram_addr ( ba0_addr   ),
    .data_dst   ( ba_dst[0]  ),
    .data_rdy   ( ba_rdy[0]  ),
    .data_read  ( data_read  )
);

jtframe_rom_1slot #(
    .SDRAMW       ( AW      ),
    .SLOT0_DW     ( 8       ),
    .SLOT0_AW     ( 18      ),
    .SLOT0_LATCH  ( 0       ),
    .SLOT0_OKLATCH( 0       ),
    .SLOT0_OFFSET ( {AW{1'b0}} )
) u_oki_rom (
    .rst        ( rst96 || dwnld_busy ),
    .clk        ( clk96               ),
    .slot0_addr ( oki_rom_addr        ),
    .slot0_dout ( oki_rom_dout        ),
    .slot0_cs   ( 1'b1                 ),
    .slot0_ok   ( oki_rom_ok           ),
    .sdram_ack  ( ba_ack[2]            ),
    .sdram_rd   ( oki_rom_rd           ),
    .sdram_addr ( oki_sdram_addr       ),
    .data_dst   ( ba_dst[2]            ),
    .data_rdy   ( ba_rdy[2]            ),
    .data_read  ( data_read            )
);

jtframe_rom_3slots #(
    .SDRAMW       ( AW      ),
    .SLOT0_DW     ( 16      ),
    .SLOT1_DW     ( 16      ),
    .SLOT2_DW     ( 16      ),
    .SLOT0_AW     ( AW      ),
    .SLOT1_AW     ( AW      ),
    .SLOT2_AW     ( AW      ),
    .SLOT0_LATCH  ( 0       ),
    .SLOT1_LATCH  ( 0       ),
    // The object line builder consumes this slot far from the SDRAM cache.
    // Registering dout lets data_ok absorb that latency and removes the
    // cache-mux-to-linebuffer critical path.
    .SLOT2_LATCH  ( 1       ),
    .SLOT0_OKLATCH( 0       ),
    .SLOT1_OKLATCH( 0       ),
    .SLOT2_OKLATCH( 0       ),
    .SLOT0_OFFSET ( {AW{1'b0}} ),
    .SLOT1_OFFSET ( {AW{1'b0}} ),
    .SLOT2_OFFSET ( {AW{1'b0}} )
) u_gfx_probe (
    .rst        ( video_runtime_reset || video_profile_gfx_reset ),
    .clk        ( clk                  ),
    .slot0_addr ( gfx_scroll_slot_addr ),
    .slot1_addr ( gfx_probe_slot_addr  ),
    .slot2_addr ( gfx_obj_slot_addr    ),
    .slot0_dout ( gfx_scroll_slot_dout ),
    .slot1_dout ( gfx_probe_slot_dout  ),
    .slot2_dout ( gfx_obj_slot_dout    ),
    .slot0_cs   ( gfx_scroll_slot_cs   ),
    .slot1_cs   ( gfx_probe_slot_cs    ),
    .slot2_cs   ( gfx_obj_slot_cs      ),
    .slot0_ok   ( gfx_scroll_slot_ok   ),
    .slot1_ok   ( gfx_probe_slot_ok    ),
    .slot2_ok   ( gfx_obj_slot_ok      ),
    .sdram_ack  ( ba_ack[1]            ),
    .sdram_rd   ( gfx_slot_rd          ),
    .sdram_addr ( gfx_sdram_addr       ),
    .data_dst   ( ba_dst[1]            ),
    .data_rdy   ( ba_rdy[1]            ),
    .data_read  ( data_read            )
);

`ifdef BATSUGUN_HW_DEBUG
wire [11:0] sound_diag_flags = {
    v25_sound_released,
    sound_diag_hold_v25,
    sound_diag_block_ym,
    sound_diag_block_oki,
    sound_diag_fault_sticky,
    sound_diag_halted_sticky,
    sound_diag_cdc_sticky,
    v25_shared_we,
    sound_debug_ym_write,
    sound_debug_oki_write,
    cpu_shared_we,
    cpu_sys_cs
};
wire [127:0] sound_diag_page0 = {
    16'h5a25,
    sound_debug_pc,
    sound_diag_flags,
    sound_diag_main_cmd_count,
    sound_diag_v25_ack_count,
    sound_diag_ym_count,
    sound_diag_oki_count,
    sound_diag_last_main_cmd,
    sound_diag_last_main_arg
};
wire [127:0] sound_diag_page1 = {
    16'h5a26,
    sound_diag_frame_count,
    cpu_addr8,
    sound_diag_last_v25_addr,
    sound_diag_last_v25_data,
    sound_diag_last_main_addr,
    sound_diag_last_main_data,
    sound_diag_last_ym_data,
    sound_diag_last_oki_data,
    sound_debug_ym_a0,
    cpu_rw
};
wire [7:0] sound_diag_main_flags = {
    sound_diag_service_commit_seen,
    sound_diag_normal_boot_seen,
    fatal_fetch_seen,
    cpu_reset,
    cpu_reset_base,
    v25_sound_released,
    ever_sound_reset,
    ever_sound_release
};
wire [127:0] sound_diag_page2 = {
    16'h5a27,
    sound_diag_frame_count,
    sound_diag_main_flags,
    sound_diag_fetch_3,
    sound_diag_fetch_2,
    sound_diag_fetch_1,
    sound_diag_fetch_0
};
wire [127:0] sound_diag_page3 = {
    16'h5a28,
    cpu_irq_sound_flags,
    fault_gp_addr,
    fault_gp_data,
    fault_gp0_latch,
    fault_gp1_latch,
    fault_bus_counts,
    sound_diag_sys_read_count,
    sound_diag_last_sys_data
};
wire [15:0] sound_diag_cpu_test_flags = {
    sound_diag_cpu_slow,
    sound_diag_cpu_wait2,
    sound_diag_cpu_wait3,
    sound_diag_fetch_data_seen,
    cpu_cen,
    cpu_cenb,
    !cpu_dtack_n,
    cpu_read,
    !cpu_lds_n,
    !cpu_uds_n,
    cpu_rw,
    cpu_bus_active
};
wire [127:0] sound_diag_page4 = {
    16'h5a29,
    sound_diag_fetch_data_3bdc2,
    sound_diag_fetch_data_3bdc4,
    sound_diag_fetch_data_3bdc6,
    sound_diag_fetch_data_3bdc8,
    sound_diag_fetch_data_3bdca,
    sound_diag_last_sys_word,
    sound_diag_cpu_test_flags
};
wire [127:0] sound_diag_page5 = {
    16'h5a2a,
    dipsw[23:0],
    sound_diag_dip_a_word,
    sound_diag_dip_b_word,
    sound_diag_region_word,
    sound_diag_dip_read_seen,
    sound_diag_boot_branch_armed,
    sound_diag_service_commit_seen,
    sound_diag_normal_boot_seen,
    cpu_sys_data,
    18'h00000
};
wire [127:0] sound_diag_page6 = {
    16'h5a2b,
    sound_diag_v25_dip_write_count,
    sound_diag_v25_dip_last_data,
    sound_diag_v25_dip_bad_seen,
    sound_diag_v25_dip_bad_data,
    sound_diag_v25_dip_bad_pc,
    sound_diag_main_dip_write_count,
    sound_diag_main_dip_write_data,
    v25_shared_addr,
    v25_shared_dout,
    v25_shared_we,
    3'b000
};
assign video_profile_probe = sound_diag_enable ?
    ((video_profile_source[2:0] == 3'd0) ? sound_diag_page0 :
     (video_profile_source[2:0] == 3'd1) ? sound_diag_page1 :
     (video_profile_source[2:0] == 3'd2) ? sound_diag_page2 :
     (video_profile_source[2:0] == 3'd3) ? sound_diag_page3 :
     (video_profile_source[2:0] == 3'd4) ? sound_diag_page4 :
     (video_profile_source[2:0] == 3'd5) ? sound_diag_page5 :
                                            sound_diag_page6) :
    video_profile_profiler_probe;

altsource_probe #(
    .sld_auto_instance_index ("NO"),
    .sld_instance_index      (0),
    .instance_id             ("BVP"),
    .probe_width             (128),
    .source_width            (32),
    .source_initial_value    ("0C000000"),
    .enable_metastability    ("NO")
) u_video_profile_jtag (
    .probe  (video_profile_probe),
    .source (video_profile_source)
);

wire [127:0] video_profile_object_trace_meta = {
    12'hb50,
    obj_debug_buffer_mode,
    gp0_scroll6_px[8:0],
    gp0_scroll7_px[8:0],
    gp0_obj_trace_attr5,
    gp0_obj_trace_code5,
    gp0_obj_trace_attr4,
    gp0_obj_trace_code4,
    gp0_obj_trace_attr3,
    gp0_obj_trace_code3
};

wire [127:0] video_profile_object_trace_motion = {
    obj_trace_frame,
    gp0_obj_trace_base_x5,
    gp0_obj_trace_base_y5,
    gp0_obj_trace_draw_x5,
    gp0_obj_trace_draw_y5,
    gp0_obj_trace_base_x4,
    gp0_obj_trace_base_y4,
    gp0_obj_trace_draw_x4,
    gp0_obj_trace_draw_y4,
    gp0_obj_trace_base_x3,
    gp0_obj_trace_base_y3,
    gp0_obj_trace_draw_x3,
    gp0_obj_trace_draw_y3
};

wire video_profile_boot_trace_enable = video_profile_source[28];
wire [127:0] video_profile_boot_trace_words = {
    rom_probe_word3,
    rom_probe_word2,
    rom_probe_word1,
    rom_probe_word0,
    load_word3,
    load_word2,
    load_word1,
    load_word0
};
wire [7:0] video_profile_boot_trace_flags = {
    rom_probe_match,
    rom_probe_done,
    load_match,
    load_done,
    cpu_reset_base,
    cpu_reset,
    v25_stub_done,
    gfx_startup_reset
};
wire [127:0] video_profile_boot_trace_state = {
    16'hb007,
    cpu_diag_flags,
    video_profile_boot_trace_flags,
    rom_probe_seen,
    load_seen,
    diag_flags,
    cpu_din,
    cpu_addr8,
    rom_slot_dout
};
wire [127:0] video_profile_trace_meta = video_profile_boot_trace_enable ?
    video_profile_boot_trace_words : video_profile_object_trace_meta;
wire [127:0] video_profile_trace_motion = video_profile_boot_trace_enable ?
    video_profile_boot_trace_state : video_profile_object_trace_motion;

batsugun_video_profiler u_video_profiler (
    .clk               (clk),
    .rst               (rst || dwnld_busy || ioctl_rom),
    .enable            (video_profile_source[3]),
    .clear_toggle      (video_profile_source[5]),
    .snapshot_toggle   (video_profile_source[4]),
    .page_sel          (video_profile_source[2:0]),
    .frame_edge        (debug_frame_tick),
    .visible           (render_lhbl && render_lvbl),
    .enqueue_mask      (video_profile_enqueue_mask),
    .overwrite_mask    (video_profile_overwrite_mask),
    .primary_launch    (video_profile_primary_launch),
    .probe_launch      (video_profile_probe_launch),
    .primary_complete  (video_profile_primary_complete),
    .probe_complete    (video_profile_probe_complete),
    .primary_deadline  (gfx_req_deadline_miss),
    .probe_deadline    (gfx_req_probe_deadline_miss),
    .stage_collision   ({video_profile_probe_stage_collision,
                         video_profile_primary_stage_collision}),
    .word_miss_mask    (comp_word_miss_mask),
    .obj_miss          (obj_prefetch_miss_event),
    .gp0_obj_write     (gp0_obj_cpu_write),
    .gp1_obj_write     (gp1_obj_cpu_write),
    .gp0_obj_copy_busy (gp0_obj_buf_busy),
    .gp1_obj_copy_busy (gp1_obj_buf_busy),
    .gp0_obj_snapshot_miss(gp0_obj_forward_miss),
    .gp1_obj_snapshot_miss(gp1_obj_forward_miss),
    .gp0_scroll_write  (gp0_scroll_cpu_write),
    .gp1_scroll_write  (gp1_scroll_cpu_write),
    .obj_buffer_mode   (obj_debug_buffer_mode),
    .object_trace_enable(video_profile_source[29] ||
                         video_profile_boot_trace_enable),
    .object_trace_meta (video_profile_trace_meta),
    .object_trace_motion(video_profile_trace_motion),
    .pending           (comp_fetch_pending),
    .stage             (comp_fetch_stage),
    .far_stage         (comp_fetch_far_stage),
    .deep_stage        (comp_fetch_deep_stage),
    .primary_pending   (gfx_req_pending),
    .primary_slot      (gfx_req_slot),
    .primary_phase     (gfx_req_phase),
    .probe_pending     (gfx_req_probe_pending),
    .probe_slot        (gfx_req_probe_slot),
    .probe_phase       (gfx_req_probe_phase),
    .hcnt              (hcnt),
    .vcnt              (vcnt),
    .clkdiv            (clkdiv),
    .slot_cs           ({gfx_obj_slot_cs, gfx_probe_slot_cs,
                         gfx_scroll_slot_cs}),
    .slot_ok           ({gfx_obj_slot_ok, gfx_probe_slot_ok,
                         gfx_scroll_slot_ok}),
    .ba1_rd            (ba_rd[1]),
    .ba1_ack           (ba_ack[1]),
    .ba1_rdy           (ba_rdy[1]),
    .gfx_reset         (video_profile_gfx_reset),
    .ioctl_rom         (ioctl_rom),
    .dwnld_busy        (dwnld_busy),
    .prog_we           (prog_we),
    .prom_we           (download_prom_we),
    .startup_hold      (gfx_startup_reset),
    .mem_probe         (video_profile_mem_probe),
    .mem_probe_data    (gfx_probe_slot_dout),
    .probe             (video_profile_profiler_probe)
);
`endif

`ifdef BATSUGUN_CPU_NO_FIXED_WAIT
localparam CPU_FIXED_WAIT = 0;
`else
localparam CPU_FIXED_WAIT = 1;
`endif

jtframe_68kdtack_cen #(.W(8), .MFREQ(94500), .WAIT1(CPU_FIXED_WAIT)) u_cpu_dtack(
    .rst        ( cpu_core_reset ),
    .clk        ( clk            ),
    .cpu_cen    ( cpu_cen        ),
    .cpu_cenb   ( cpu_cenb       ),
    .bus_cs     ( cpu_bus_cs     ),
    .bus_busy   ( cpu_bus_busy   ),
    .bus_legit  ( 1'b0           ),
    .bus_ack    ( 1'b0           ),
    .ASn        ( cpu_as_n       ),
    .DSn        ( {cpu_uds_n, cpu_lds_n} ),
    .num        ( sound_diag_cpu_slow ? 7'd16 : 7'd32 ),
    .den        ( 8'd189         ),
    .wait2      ( sound_diag_cpu_wait2 ),
    .wait3      ( sound_diag_cpu_wait3 ),
    .DTACKn     ( cpu_dtack_n    ),
    .fave       (                ),
    .fworst     (                )
);

fx68k u_main68k (
    .clk        ( clk            ),
    .HALTn      ( 1'b1           ),
    .extReset   ( cpu_core_reset ),
    .pwrUp      ( cpu_core_reset ),
    .enPhi1     ( cpu_cen && ss_cpu_run  ),
    .enPhi2     ( cpu_cenb && ss_cpu_run ),
    .eRWn       ( cpu_rw         ),
    .ASn        ( cpu_as_n       ),
    .LDSn       ( cpu_lds_n      ),
    .UDSn       ( cpu_uds_n      ),
    .E          (                ),
    .VMAn       (                ),
    .FC0        ( cpu_fc0        ),
    .FC1        ( cpu_fc1        ),
    .FC2        ( cpu_fc2        ),
    .BGn        ( cpu_bg_n       ),
    .oRESETn    (                ),
    .oHALTEDn   (                ),
    .DTACKn     ( cpu_dtack_n    ),
    .VPAn       ( cpu_vpa_n      ),
    .BERRn      ( 1'b1           ),
    .BRn        ( 1'b1           ),
    .BGACKn     ( 1'b1           ),
    .IPL0n      ( ~ss_irq        ),
    .IPL1n      ( ~ss_irq        ),
    .IPL2n      ( ~(ss_irq || cpu_irq4) ),
    .iEdb       ( cpu_din        ),
    .oEdb       ( cpu_dout       ),
    .eab        ( cpu_addr       )
);

wire [14:0] ss_wram_addr;
wire [15:0] ss_wram_data;
wire [ 1:0] ss_wram_we;
wire [15:0] ss_wram_q;

batsugun_ss_ram_port #(
    .WIDTH        ( 16   ),
    .ADDR_WIDTH   ( 15   ),
    .WE_WIDTH     ( 2    ),
    .SS_IDX       ( 8'd2 ),
    .STREAM_WIDTH ( 2'd1 )
) u_wram_ss (
    .clk            ( clk                   ),
    .restore_enable ( ss_restore_compatible ),
    .normal_we      ( 2'b00                 ),
    .normal_addr    ( 15'd0                 ),
    .normal_data    ( 16'd0                 ),
    .ram_we         ( ss_wram_we            ),
    .ram_addr       ( ss_wram_addr          ),
    .ram_data       ( ss_wram_data          ),
    .ram_q          ( ss_wram_q             ),
    .ss_data        ( ss_data               ),
    .ss_addr        ( ss_addr               ),
    .ss_select      ( ss_select             ),
    .ss_write       ( ss_write              ),
    .ss_read        ( ss_read               ),
    .ss_query       ( ss_query              ),
    .ss_data_out    ( ss_wram_data_out      ),
    .ss_ack         ( ss_wram_ack           )
);

jtframe_dual_ram16 #(.AW(15)) u_cpu_wram(
    .clk0       ( clk            ),
    .data0      ( cpu_dout       ),
    .addr0      ( cpu_addr[15:1] ),
    .we0        ( cpu_wram_we    ),
    .q0         ( cpu_wram_q     ),
    .clk1       ( clk            ),
    .data1      ( ss_wram_data   ),
    .addr1      ( ss_wram_addr   ),
    .we1        ( ss_wram_we     ),
    .q1         ( ss_wram_q      )
);

wire [14:0] ss_shared_addr;
wire [ 7:0] ss_shared_data;
wire [ 0:0] ss_shared_we;

batsugun_ss_ram_port #(
    .WIDTH        ( 8    ),
    .ADDR_WIDTH   ( 15   ),
    .WE_WIDTH     ( 1    ),
    .SS_IDX       ( 8'd3 ),
    .STREAM_WIDTH ( 2'd0 )
) u_shared_ss (
    .clk            ( clk                   ),
    .restore_enable ( ss_restore_compatible ),
    .normal_we      ( v25_preclear_we || v25_shared_we ),
    .normal_addr    ( v25_preclear_we ? v25_stub_addr : v25_shared_addr ),
    .normal_data    ( v25_preclear_we ? 8'h00 : v25_shared_dout ),
    .ram_we         ( ss_shared_we          ),
    .ram_addr       ( ss_shared_addr        ),
    .ram_data       ( ss_shared_data        ),
    .ram_q          ( v25_shared_din        ),
    .ss_data        ( ss_data               ),
    .ss_addr        ( ss_addr               ),
    .ss_select      ( ss_select             ),
    .ss_write       ( ss_write              ),
    .ss_read        ( ss_read               ),
    .ss_query       ( ss_query              ),
    .ss_data_out    ( ss_shared_data_out    ),
    .ss_ack         ( ss_shared_ack         )
);

jtframe_dual_ram #(.DW(8), .AW(15)) u_shared_ram(
    .clk0       ( clk            ),
    .data0      ( cpu_shared_din ),
    .addr0      ( cpu_addr[15:1] ),
    .we0        ( cpu_shared_we  ),
    .q0         ( cpu_shared_q   ),
    .clk1       ( clk96          ),
    .data1      ( ss_shared_data ),
    .addr1      ( ss_shared_addr ),
    .we1        ( ss_shared_we[0] ),
    .q1         ( v25_shared_din )
);

wire v25_boot_shadow_capture =
    cpu_shared_we &&
    !v25_sound_released &&
    (cpu_addr8[15:1] >= 15'h7e00);

// The V25 erases its reset trampoline after boot. Preserve the 512-byte page
// written by the 68000 so a save-state restore can reconstruct a cold boot.
jtframe_dual_ram #(.DW(8), .AW(9)) u_v25_boot_shadow(
    .clk0       ( clk                    ),
    .data0      ( cpu_shared_din         ),
    .addr0      ( cpu_addr8[9:1]         ),
    .we0        ( v25_boot_shadow_capture),
    .q0         (                        ),
    .clk1       ( clk96                  ),
    .data1      ( 8'd0                   ),
    .addr1      ( v25_boot_shadow_addr   ),
    .we1        ( 1'b0                   ),
    .q1         ( v25_boot_shadow_data   )
);

batsugun_sound u_sound (
    .reset              ( rst96 || dwnld_busy || !v25_stub_done ),
    .clk16              ( clk96                   ),
    .v25_cen            ( v25_cen && ss_v25_run   ),
    .clk_sound          ( clk96                   ),
    .v25_enable         ( v25_sound_released      ),
    .dip_a              ( dipsw[7:0]              ),
    .dip_b              ( dipsw[15:8]             ),
    .region             ( dipsw[23:16]            ),
    .debug_hold_v25     ( sound_diag_hold_v25     ),
    .debug_block_ym     ( sound_diag_block_ym     ),
    .debug_block_oki    ( sound_diag_block_oki    ),
    .ym_enable          ( snd_en[0] && !status[9]  ),
    .oki_enable         ( snd_en[1] && !status[8]  ),
    .ss_hold            ( ss_device_hold           ),
    .ss_restore_enable  ( ss_restore_compatible   ),
    .ss_restore_commit  ( ss_restore_commit       ),
    .ss_restore_bgm_valid( ss_restore_bgm_valid   ),
    .ss_restore_bgm_command( ss_restore_bgm_command ),
    .ss_restore_bgm_argument( ss_restore_bgm_argument ),
    .ss_data            ( ss_data                 ),
    .ss_addr            ( ss_addr                 ),
    .ss_select          ( ss_select               ),
    .ss_write           ( ss_write                ),
    .ss_read            ( ss_read                 ),
    .ss_query           ( ss_query                ),
    .ss_data_out        ( ss_v25_data_out         ),
    .ss_ack             ( ss_v25_ack              ),
    .v25_state_idle     ( sound_v25_state_idle    ),
    .sound_state_idle   ( sound_state_idle        ),
    .sound_state_held   ( sound_state_held        ),
    .shared_addr        ( v25_shared_addr         ),
    .shared_dout        ( v25_shared_dout         ),
    .shared_we          ( v25_shared_we           ),
    .shared_din         ( v25_shared_din          ),
    .boot_shadow_addr   ( v25_boot_shadow_addr    ),
    .boot_shadow_data   ( v25_boot_shadow_data    ),
    .oki_rom_addr       ( oki_rom_addr            ),
    .oki_rom_data       ( oki_rom_dout            ),
    .oki_rom_ok         ( oki_rom_ok              ),
    .snd_mono           ( sound_mono              ),
    .sample             ( sound_sample            ),
    .debug_fault        ( sound_debug_fault       ),
    .debug_halted       ( sound_debug_halted      ),
    .debug_pc           ( sound_debug_pc          ),
    .debug_ym_write     ( sound_debug_ym_write    ),
    .debug_ym_a0        ( sound_debug_ym_a0       ),
    .debug_ym_data      ( sound_debug_ym_data     ),
    .debug_oki_write    ( sound_debug_oki_write   ),
    .debug_oki_data     ( sound_debug_oki_data    ),
    .debug_cdc_overrun  ( sound_debug_cdc_overrun )
);

`ifdef BATSUGUN_HW_DEBUG
always @(posedge clk) begin
    if (cpu_reset) begin
        sound_diag_frame_count <= 24'h000000;
        sound_diag_main_cmd_count <= 16'h0000;
        sound_diag_last_main_cmd <= 8'h00;
        sound_diag_last_main_arg <= 8'h00;
        sound_diag_last_main_addr <= 15'h0000;
        sound_diag_last_main_data <= 8'h00;
        sound_diag_sys_read_count <= 8'h00;
        sound_diag_last_sys_data <= 8'h00;
        sound_diag_service_commit_seen <= 1'b0;
        sound_diag_normal_boot_seen <= 1'b0;
        sound_diag_boot_branch_armed <= 1'b0;
        sound_diag_fetch_0 <= 20'h00000;
        sound_diag_fetch_1 <= 20'h00000;
        sound_diag_fetch_2 <= 20'h00000;
        sound_diag_fetch_3 <= 20'h00000;
        sound_diag_fetch_data_3bdc2 <= 16'h0000;
        sound_diag_fetch_data_3bdc4 <= 16'h0000;
        sound_diag_fetch_data_3bdc6 <= 16'h0000;
        sound_diag_fetch_data_3bdc8 <= 16'h0000;
        sound_diag_fetch_data_3bdca <= 16'h0000;
        sound_diag_last_sys_word <= 16'h0000;
        sound_diag_fetch_data_seen <= 5'b00000;
        sound_diag_dip_a_word <= 16'h0000;
        sound_diag_dip_b_word <= 16'h0000;
        sound_diag_region_word <= 16'h0000;
        sound_diag_dip_read_seen <= 3'b000;
        sound_diag_main_dip_write_count <= 16'h0000;
        sound_diag_main_dip_write_data <= 16'h0000;
    end else begin
        if (debug_frame_tick)
            sound_diag_frame_count <= sound_diag_frame_count + 24'd1;
        if (cpu_ack_now && cpu_sys_cs) begin
            sound_diag_sys_read_count <= sound_diag_sys_read_count + 8'd1;
            sound_diag_last_sys_data <= cpu_din[7:0];
            sound_diag_last_sys_word <= cpu_din;
        end
        if (cpu_ack_now && cpu_shared_cs && cpu_read) begin
            case (cpu_addr8)
                24'h21f004: begin
                    sound_diag_dip_a_word <= cpu_din;
                    sound_diag_dip_read_seen[2] <= 1'b1;
                end
                24'h21f006: begin
                    sound_diag_dip_b_word <= cpu_din;
                    sound_diag_dip_read_seen[1] <= 1'b1;
                end
                24'h21f008: begin
                    sound_diag_region_word <= cpu_din;
                    sound_diag_dip_read_seen[0] <= 1'b1;
                end
                default: begin end
            endcase
        end
        if (cpu_ack_now && cpu_shared_we &&
            cpu_addr8 == 24'h21f004) begin
            sound_diag_main_dip_write_count <=
                sound_diag_main_dip_write_count + 16'd1;
            sound_diag_main_dip_write_data <= cpu_dout;
        end
        if (cpu_rom_ack) begin
            case (cpu_addr8)
                24'h03bdc2: begin
                    sound_diag_fetch_data_3bdc2 <= cpu_din;
                    sound_diag_fetch_data_seen[4] <= 1'b1;
                end
                24'h03bdc4: begin
                    sound_diag_fetch_data_3bdc4 <= cpu_din;
                    sound_diag_fetch_data_seen[3] <= 1'b1;
                end
                24'h03bdc6: begin
                    sound_diag_fetch_data_3bdc6 <= cpu_din;
                    sound_diag_fetch_data_seen[2] <= 1'b1;
                end
                24'h03bdc8: begin
                    sound_diag_fetch_data_3bdc8 <= cpu_din;
                    sound_diag_fetch_data_seen[1] <= 1'b1;
                end
                24'h03bdca: begin
                    sound_diag_fetch_data_3bdca <= cpu_din;
                    sound_diag_fetch_data_seen[0] <= 1'b1;
                end
                default: begin end
            endcase
        end
        if (!sound_diag_service_commit_seen && !sound_diag_normal_boot_seen) begin
            if (cpu_boot_branch_fetch)
                sound_diag_boot_branch_armed <= 1'b1;
            if (sound_diag_boot_branch_armed && cpu_service_commit) begin
                sound_diag_service_commit_seen <= 1'b1;
                sound_diag_boot_branch_armed <= 1'b0;
            end else if (sound_diag_boot_branch_armed &&
                         cpu_normal_boot_fetch) begin
                sound_diag_normal_boot_seen <= 1'b1;
                sound_diag_boot_branch_armed <= 1'b0;
            end
        end
        if (!sound_diag_service_commit_seen && !fatal_fetch_seen) begin
            if (!cpu_fatal_fetch && cpu_nonfatal_rom_fetch) begin
                sound_diag_fetch_3 <= sound_diag_fetch_2;
                sound_diag_fetch_2 <= sound_diag_fetch_1;
                sound_diag_fetch_1 <= sound_diag_fetch_0;
                sound_diag_fetch_0 <= cpu_addr8[19:0];
            end
        end
        if (cpu_ack_now && cpu_shared_we) begin
            sound_diag_last_main_addr <= cpu_addr8[15:1];
            sound_diag_last_main_data <= cpu_shared_din;
            if (cpu_addr8[15:1] == 15'h7801)
                sound_diag_last_main_arg <= cpu_shared_din;
            if (cpu_addr8[15:1] == 15'h7800 &&
                cpu_shared_din != 8'hff) begin
                sound_diag_last_main_cmd <= cpu_shared_din;
                sound_diag_main_cmd_count <=
                    sound_diag_main_cmd_count + 16'd1;
            end
        end
    end
end

always @(posedge clk96) begin
    if (rst96 || dwnld_busy || !v25_stub_done) begin
        sound_diag_v25_ack_count <= 16'h0000;
        sound_diag_ym_count <= 16'h0000;
        sound_diag_oki_count <= 16'h0000;
        sound_diag_last_v25_addr <= 15'h0000;
        sound_diag_last_v25_data <= 8'h00;
        sound_diag_last_ym_data <= 8'h00;
        sound_diag_last_oki_data <= 8'h00;
        sound_diag_fault_sticky <= 1'b0;
        sound_diag_halted_sticky <= 1'b0;
        sound_diag_cdc_sticky <= 1'b0;
        sound_diag_v25_dip_write_count <= 16'h0000;
        sound_diag_v25_dip_last_data <= 8'h00;
        sound_diag_v25_dip_bad_seen <= 1'b0;
        sound_diag_v25_dip_bad_data <= 8'h00;
        sound_diag_v25_dip_bad_pc <= 20'h00000;
    end else begin
        if (v25_shared_we) begin
            sound_diag_last_v25_addr <= v25_shared_addr;
            sound_diag_last_v25_data <= v25_shared_dout;
            if (v25_shared_addr == 15'h7800 &&
                v25_shared_dout == 8'hff)
                sound_diag_v25_ack_count <=
                    sound_diag_v25_ack_count + 16'd1;
            if (v25_shared_addr == 15'h7802) begin
                sound_diag_v25_dip_write_count <=
                    sound_diag_v25_dip_write_count + 16'd1;
                sound_diag_v25_dip_last_data <= v25_shared_dout;
                if (v25_shared_dout != 8'h00 &&
                    !sound_diag_v25_dip_bad_seen) begin
                    sound_diag_v25_dip_bad_seen <= 1'b1;
                    sound_diag_v25_dip_bad_data <= v25_shared_dout;
                    sound_diag_v25_dip_bad_pc <= sound_debug_pc;
                end
            end
        end
        if (sound_debug_ym_write) begin
            sound_diag_ym_count <= sound_diag_ym_count + 16'd1;
            sound_diag_last_ym_data <= sound_debug_ym_data;
        end
        if (sound_debug_oki_write) begin
            sound_diag_oki_count <= sound_diag_oki_count + 16'd1;
            sound_diag_last_oki_data <= sound_debug_oki_data;
        end
        if (v25_sound_released && !sound_diag_hold_v25) begin
            if (sound_debug_fault)
                sound_diag_fault_sticky <= 1'b1;
            if (sound_debug_halted)
                sound_diag_halted_sticky <= 1'b1;
            if (sound_debug_cdc_overrun)
                sound_diag_cdc_sticky <= 1'b1;
        end
    end
end
`endif

wire [11:0] ss_palette_addr;
wire [15:0] ss_palette_data;
wire [ 1:0] ss_palette_we;

batsugun_ss_ram_port #(
    .WIDTH        ( 16   ),
    .ADDR_WIDTH   ( 12   ),
    .WE_WIDTH     ( 2    ),
    .SS_IDX       ( 8'd4 ),
    .STREAM_WIDTH ( 2'd1 )
) u_palette_ss (
    .clk            ( clk                   ),
    .restore_enable ( ss_restore_compatible ),
    .normal_we      ( 2'b00                 ),
    .normal_addr    ( pal_scan_addr         ),
    .normal_data    ( 16'd0                 ),
    .ram_we         ( ss_palette_we         ),
    .ram_addr       ( ss_palette_addr       ),
    .ram_data       ( ss_palette_data       ),
    .ram_q          ( pal_scan_q            ),
    .ss_data        ( ss_data               ),
    .ss_addr        ( ss_addr               ),
    .ss_select      ( ss_select             ),
    .ss_write       ( ss_write              ),
    .ss_read        ( ss_read               ),
    .ss_query       ( ss_query              ),
    .ss_data_out    ( ss_palette_data_out   ),
    .ss_ack         ( ss_palette_ack        )
);

jtframe_dual_ram16 #(.AW(12)) u_palette_ram(
    .clk0       ( clk            ),
    .data0      ( cpu_dout       ),
    .addr0      ( cpu_addr[12:1] ),
    .we0        ( cpu_pal_we     ),
    .q0         ( cpu_pal_q      ),
    .clk1       ( clk            ),
    .data1      ( ss_palette_data ),
    .addr1      ( ss_palette_addr ),
    .we1        ( ss_palette_we   ),
    .q1         ( pal_scan_q     )
);

batsugun_gp9001_stub #(
    .SS_RAM_IDX  ( 8'd5  ),
    .SS_REG_IDX  ( 8'd7  ),
    .SS_OBJ0_IDX ( 8'd9  ),
    .SS_OBJ1_IDX ( 8'd10 )
) u_gp9001_0 (
    .clk          ( clk                 ),
    .rst          ( cpu_reset           ),
    .start        ( gp0_start           ),
    .rw           ( cpu_rw              ),
    .addr         ( cpu_addr8[3:0]      ),
    .din          ( cpu_dout            ),
    .we_mask      ( {!cpu_uds_n, !cpu_lds_n} ),
    .status_bit   ( cpu_gp_status       ),
    .scan_addr    ( gp0_scan_addr       ),
    .scan_dout    ( gp0_scan_dout       ),
    .obj_scan_addr( gp0_obj_scan_addr   ),
    .obj_scan_dout( gp0_obj_scan_dout   ),
    .obj_buf_start ( obj_buffer_copy_start ),
    .obj_buf_lock  ( obj_cache_scan_start_tick ),
    .obj_scan_live ( obj_scan_live_enable ),
    .obj_buf_busy  ( gp0_obj_buf_busy      ),
    .obj_forward_active(gp0_obj_forward_active),
    .obj_forward_write (gp0_obj_forward_write),
    .obj_forward_miss  (gp0_obj_forward_miss),
    .busy         ( gp0_busy            ),
    .done         ( gp0_done            ),
    .dout         ( gp0_dout            ),
    .irq_clear    ( gp0_irq_clear       ),
    .dbg_ptr      ( gp0_dbg_ptr         ),
    .dbg_last_addr( gp0_dbg_last_addr   ),
    .dbg_last_din ( gp0_dbg_last_din    ),
    .dbg_last_dout( gp0_dbg_last_dout   ),
    .dbg_regs_01  ( gp0_dbg_regs_01     ),
    .dbg_regs_23  ( gp0_dbg_regs_23     ),
    .scroll0      ( gp0_scroll0         ),
    .scroll1      ( gp0_scroll1         ),
    .scroll2      ( gp0_scroll2         ),
    .scroll3      ( gp0_scroll3         ),
    .scroll4      ( gp0_scroll4         ),
    .scroll5      ( gp0_scroll5         ),
    .scroll6      ( gp0_scroll6         ),
    .scroll7      ( gp0_scroll7         ),
    .ss_hold      ( ss_device_hold      ),
    .ss_restore_enable(ss_restore_compatible),
    .ss_data      ( ss_data             ),
    .ss_addr      ( ss_addr             ),
    .ss_select    ( ss_select           ),
    .ss_write     ( ss_write            ),
    .ss_read      ( ss_read             ),
    .ss_query     ( ss_query            ),
    .ss_data_out  ( ss_gp0_data_out     ),
    .ss_ack       ( ss_gp0_ack          )
);

batsugun_gp9001_stub #(
    .SS_RAM_IDX  ( 8'd6  ),
    .SS_REG_IDX  ( 8'd8  ),
    .SS_OBJ0_IDX ( 8'd11 ),
    .SS_OBJ1_IDX ( 8'd12 )
) u_gp9001_1 (
    .clk          ( clk                 ),
    .rst          ( cpu_reset           ),
    .start        ( gp1_start           ),
    .rw           ( cpu_rw              ),
    .addr         ( cpu_addr8[3:0]      ),
    .din          ( cpu_dout            ),
    .we_mask      ( {!cpu_uds_n, !cpu_lds_n} ),
    .status_bit   ( cpu_gp_status       ),
    .scan_addr    ( gp1_scan_addr       ),
    .scan_dout    ( gp1_scan_dout       ),
    .obj_scan_addr( gp1_obj_scan_addr   ),
    .obj_scan_dout( gp1_obj_scan_dout   ),
    .obj_buf_start ( obj_buffer_copy_start ),
    .obj_buf_lock  ( obj_cache_scan_start_tick ),
    .obj_scan_live ( obj_scan_live_enable ),
    .obj_buf_busy  ( gp1_obj_buf_busy      ),
    .obj_forward_active(gp1_obj_forward_active),
    .obj_forward_write (gp1_obj_forward_write),
    .obj_forward_miss  (gp1_obj_forward_miss),
    .busy         ( gp1_busy            ),
    .done         ( gp1_done            ),
    .dout         ( gp1_dout            ),
    .irq_clear    ( gp1_irq_clear       ),
    .dbg_ptr      ( gp1_dbg_ptr         ),
    .dbg_last_addr( gp1_dbg_last_addr   ),
    .dbg_last_din ( gp1_dbg_last_din    ),
    .dbg_last_dout( gp1_dbg_last_dout   ),
    .dbg_regs_01  ( gp1_dbg_regs_01     ),
    .dbg_regs_23  ( gp1_dbg_regs_23     ),
    .scroll0      ( gp1_scroll0         ),
    .scroll1      ( gp1_scroll1         ),
    .scroll2      ( gp1_scroll2         ),
    .scroll3      ( gp1_scroll3         ),
    .scroll4      ( gp1_scroll4         ),
    .scroll5      ( gp1_scroll5         ),
    .scroll6      ( gp1_scroll6         ),
    .scroll7      ( gp1_scroll7         ),
    .ss_hold      ( ss_device_hold      ),
    .ss_restore_enable(ss_restore_compatible),
    .ss_data      ( ss_data             ),
    .ss_addr      ( ss_addr             ),
    .ss_select    ( ss_select           ),
    .ss_write     ( ss_write            ),
    .ss_read      ( ss_read             ),
    .ss_query     ( ss_query            ),
    .ss_data_out  ( ss_gp1_data_out     ),
    .ss_ack       ( ss_gp1_ack          )
);

always @(posedge clk) begin
    if (rst || dwnld_busy || ioctl_rom) begin
        gp0_scroll0_px <= 16'h0000;
        gp0_scroll1_px <= 16'h0000;
        gp0_scroll2_px <= 16'h0000;
        gp0_scroll3_px <= 16'h0000;
        gp0_scroll4_px <= 16'h0000;
        gp0_scroll5_px <= 16'h0000;
        gp0_scroll6_px <= 16'h0000;
        gp0_scroll7_px <= 16'h0000;
        gp1_scroll0_px <= 16'h0000;
        gp1_scroll1_px <= 16'h0000;
        gp1_scroll2_px <= 16'h0000;
        gp1_scroll3_px <= 16'h0000;
        gp1_scroll4_px <= 16'h0000;
        gp1_scroll5_px <= 16'h0000;
        gp1_scroll6_px <= 16'h0000;
        gp1_scroll7_px <= 16'h0000;
    end else if ((clkdiv == 4'd13) && !render_lvbl) begin
        gp0_scroll0_px <= gp0_scroll0;
        gp0_scroll1_px <= gp0_scroll1;
        gp0_scroll2_px <= gp0_scroll2;
        gp0_scroll3_px <= gp0_scroll3;
        gp0_scroll4_px <= gp0_scroll4;
        gp0_scroll5_px <= gp0_scroll5;
        gp0_scroll6_px <= gp0_scroll6;
        gp0_scroll7_px <= gp0_scroll7;
        gp1_scroll0_px <= gp1_scroll0;
        gp1_scroll1_px <= gp1_scroll1;
        gp1_scroll2_px <= gp1_scroll2;
        gp1_scroll3_px <= gp1_scroll3;
        gp1_scroll4_px <= gp1_scroll4;
        gp1_scroll5_px <= gp1_scroll5;
        gp1_scroll6_px <= gp1_scroll6;
        gp1_scroll7_px <= gp1_scroll7;
    end
end

always @(posedge clk) begin
    if (cpu_reset) begin
        obj_scan_idx <= 8'h00;
        obj_scan_phase <= 4'd0;
        obj_cache_scan_active <= 1'b0;
`ifdef BATSUGUN_HW_DEBUG
        obj_trace_frame <= 20'h00000;
        gp0_obj_trace_attr3 <= 16'h0000;
        gp0_obj_trace_code3 <= 16'h0000;
        gp0_obj_trace_base_x3 <= 9'h000;
        gp0_obj_trace_base_y3 <= 9'h000;
        gp0_obj_trace_draw_x3 <= 9'h000;
        gp0_obj_trace_draw_y3 <= 9'h000;
        gp0_obj_trace_attr4 <= 16'h0000;
        gp0_obj_trace_code4 <= 16'h0000;
        gp0_obj_trace_base_x4 <= 9'h000;
        gp0_obj_trace_base_y4 <= 9'h000;
        gp0_obj_trace_draw_x4 <= 9'h000;
        gp0_obj_trace_draw_y4 <= 9'h000;
        gp0_obj_trace_attr5 <= 16'h0000;
        gp0_obj_trace_code5 <= 16'h0000;
        gp0_obj_trace_base_x5 <= 9'h000;
        gp0_obj_trace_base_y5 <= 9'h000;
        gp0_obj_trace_draw_x5 <= 9'h000;
        gp0_obj_trace_draw_y5 <= 9'h000;
`endif
        gp0_obj_scan_addr <= GP_OBJ_BASE;
        gp1_obj_scan_addr <= GP_OBJ_BASE;
        gp0_obj_word0 <= 16'h0000;
        gp0_obj_word1 <= 16'h0000;
        gp0_obj_word2 <= 16'h0000;
        gp0_obj_word3 <= 16'h0000;
        gp1_obj_word0 <= 16'h0000;
        gp1_obj_word1 <= 16'h0000;
        gp1_obj_word2 <= 16'h0000;
        gp1_obj_word3 <= 16'h0000;
        gp0_obj_old_x <= 9'd0;
        gp0_obj_old_y <= 9'd0;
        gp1_obj_old_x <= 9'd0;
        gp1_obj_old_y <= 9'd0;
        gp0_obj_stage_live <= 1'b0;
        gp1_obj_stage_live <= 1'b0;
        gp0_obj_stage_base_x <= 9'd0;
        gp0_obj_stage_base_y <= 9'd0;
        gp1_obj_stage_base_x <= 9'd0;
        gp1_obj_stage_base_y <= 9'd0;
        gp0_obj_stage_draw_x <= 9'd0;
        gp0_obj_stage_draw_y <= 9'd0;
        gp1_obj_stage_draw_x <= 9'd0;
        gp1_obj_stage_draw_y <= 9'd0;
        gp0_obj_stage_w <= 8'd0;
        gp0_obj_stage_h <= 8'd0;
        gp1_obj_stage_w <= 8'd0;
        gp1_obj_stage_h <= 8'd0;
        gp0_obj_count <= 7'd0;
        gp1_obj_count <= 7'd0;
        gp0_obj_live_total <= 8'h00;
        gp1_obj_live_total <= 8'h00;
        gp0_obj_visible_total <= 8'h00;
        gp1_obj_visible_total <= 8'h00;
        gp0_obj_visible_overflow_total <= 8'h00;
        gp1_obj_visible_overflow_total <= 8'h00;
        gp0_obj_live_latched <= 8'h00;
        gp1_obj_live_latched <= 8'h00;
        gp0_obj_visible_latched <= 8'h00;
        gp1_obj_visible_latched <= 8'h00;
        gp0_obj_cached_latched <= 8'h00;
        gp1_obj_cached_latched <= 8'h00;
        gp0_obj_visible_overflow_latched <= 8'h00;
        gp1_obj_visible_overflow_latched <= 8'h00;
        for (obj_reset_i = 0; obj_reset_i < OBJ_AUX_COUNT; obj_reset_i = obj_reset_i + 1) begin
            gp0_obj_x[obj_reset_i] <= 9'd0;
            gp0_obj_y[obj_reset_i] <= 9'd0;
            gp0_obj_raw_base_x[obj_reset_i] <= 9'd0;
            gp0_obj_raw_base_y[obj_reset_i] <= 9'd0;
            gp0_obj_w[obj_reset_i] <= 8'd0;
            gp0_obj_h[obj_reset_i] <= 8'd0;
            gp0_obj_attr[obj_reset_i] <= 16'h0000;
            gp0_obj_code[obj_reset_i] <= 18'h00000;
            gp0_obj_raw_x[obj_reset_i] <= 16'h0000;
            gp0_obj_raw_y[obj_reset_i] <= 16'h0000;
            gp1_obj_x[obj_reset_i] <= 9'd0;
            gp1_obj_y[obj_reset_i] <= 9'd0;
            gp1_obj_raw_base_x[obj_reset_i] <= 9'd0;
            gp1_obj_raw_base_y[obj_reset_i] <= 9'd0;
            gp1_obj_w[obj_reset_i] <= 8'd0;
            gp1_obj_h[obj_reset_i] <= 8'd0;
            gp1_obj_attr[obj_reset_i] <= 16'h0000;
            gp1_obj_code[obj_reset_i] <= 18'h00000;
            gp1_obj_raw_x[obj_reset_i] <= 16'h0000;
            gp1_obj_raw_y[obj_reset_i] <= 16'h0000;
        end
    end else if (obj_cache_scan_start_tick) begin
        obj_scan_idx <= 8'h00;
        obj_scan_phase <= 4'd0;
        obj_cache_scan_active <= 1'b1;
`ifdef BATSUGUN_HW_DEBUG
        obj_trace_frame <= obj_trace_frame + 20'd1;
`endif
        gp0_obj_scan_addr <= GP_OBJ_BASE;
        gp1_obj_scan_addr <= GP_OBJ_BASE;
        gp0_obj_old_x <= 9'd0;
        gp0_obj_old_y <= 9'd0;
        gp1_obj_old_x <= 9'd0;
        gp1_obj_old_y <= 9'd0;
        gp0_obj_stage_live <= 1'b0;
        gp1_obj_stage_live <= 1'b0;
        gp0_obj_count <= 7'd0;
        gp1_obj_count <= 7'd0;
        gp0_obj_live_total <= 8'h00;
        gp1_obj_live_total <= 8'h00;
        gp0_obj_visible_total <= 8'h00;
        gp1_obj_visible_total <= 8'h00;
        gp0_obj_visible_overflow_total <= 8'h00;
        gp1_obj_visible_overflow_total <= 8'h00;
    end else begin
        case (obj_scan_phase)
            4'd0: begin
                gp0_obj_scan_addr <= GP_OBJ_BASE + {3'b000, obj_scan_idx, 2'b00};
                gp1_obj_scan_addr <= GP_OBJ_BASE + {3'b000, obj_scan_idx, 2'b00};
                obj_scan_phase <= 4'd1;
            end
            4'd1: begin
                obj_scan_phase <= 4'd2;
            end
            4'd2: begin
                gp0_obj_word0 <= gp0_obj_scan_dout;
                gp1_obj_word0 <= gp1_obj_scan_dout;
                gp0_obj_scan_addr <= GP_OBJ_BASE + {3'b000, obj_scan_idx, 2'b00} + 13'd1;
                gp1_obj_scan_addr <= GP_OBJ_BASE + {3'b000, obj_scan_idx, 2'b00} + 13'd1;
                obj_scan_phase <= 4'd3;
            end
            4'd3: begin
                obj_scan_phase <= 4'd4;
            end
            4'd4: begin
                gp0_obj_word1 <= gp0_obj_scan_dout;
                gp1_obj_word1 <= gp1_obj_scan_dout;
                gp0_obj_scan_addr <= GP_OBJ_BASE + {3'b000, obj_scan_idx, 2'b00} + 13'd2;
                gp1_obj_scan_addr <= GP_OBJ_BASE + {3'b000, obj_scan_idx, 2'b00} + 13'd2;
                obj_scan_phase <= 4'd5;
            end
            4'd5: begin
                obj_scan_phase <= 4'd6;
            end
            4'd6: begin
                gp0_obj_word2 <= gp0_obj_scan_dout;
                gp1_obj_word2 <= gp1_obj_scan_dout;
                gp0_obj_scan_addr <= GP_OBJ_BASE + {3'b000, obj_scan_idx, 2'b00} + 13'd3;
                gp1_obj_scan_addr <= GP_OBJ_BASE + {3'b000, obj_scan_idx, 2'b00} + 13'd3;
                obj_scan_phase <= 4'd7;
            end
            4'd7: begin
                obj_scan_phase <= 4'd8;
            end
            4'd8: begin
                gp0_obj_word3 <= gp0_obj_scan_dout;
                gp1_obj_word3 <= gp1_obj_scan_dout;
                obj_scan_phase <= 4'd9;
            end
            4'd9: begin
                gp0_obj_stage_live <= gp0_obj_live_candidate;
                gp1_obj_stage_live <= gp1_obj_live_candidate;
                gp0_obj_stage_base_x <= gp0_obj_base_x;
                gp0_obj_stage_base_y <= gp0_obj_base_y;
                gp1_obj_stage_base_x <= gp1_obj_base_x;
                gp1_obj_stage_base_y <= gp1_obj_base_y;
                gp0_obj_stage_draw_x <= gp0_obj_draw_x;
                gp0_obj_stage_draw_y <= gp0_obj_draw_y;
                gp1_obj_stage_draw_x <= gp1_obj_draw_x;
                gp1_obj_stage_draw_y <= gp1_obj_draw_y;
                gp0_obj_stage_w <= gp0_obj_w_calc;
                gp0_obj_stage_h <= gp0_obj_h_calc;
                gp1_obj_stage_w <= gp1_obj_w_calc;
                gp1_obj_stage_h <= gp1_obj_h_calc;
                obj_scan_phase <= 4'd10;
            end
            4'd10: begin
                gp0_obj_live_total <= gp0_obj_live_total_next;
                gp1_obj_live_total <= gp1_obj_live_total_next;
                gp0_obj_visible_total <= gp0_obj_visible_total_next;
                gp1_obj_visible_total <= gp1_obj_visible_total_next;
                gp0_obj_visible_overflow_total <= gp0_obj_visible_overflow_total_next;
                gp1_obj_visible_overflow_total <= gp1_obj_visible_overflow_total_next;

`ifdef BATSUGUN_HW_DEBUG
                case (obj_scan_idx)
                    8'd3: begin
                        gp0_obj_trace_attr3 <= gp0_obj_word0;
                        gp0_obj_trace_code3 <= gp0_obj_word1;
                        gp0_obj_trace_base_x3 <= gp0_obj_stage_base_x;
                        gp0_obj_trace_base_y3 <= gp0_obj_stage_base_y;
                        gp0_obj_trace_draw_x3 <= gp0_obj_stage_draw_x;
                        gp0_obj_trace_draw_y3 <= gp0_obj_stage_draw_y;
                    end
                    8'd4: begin
                        gp0_obj_trace_attr4 <= gp0_obj_word0;
                        gp0_obj_trace_code4 <= gp0_obj_word1;
                        gp0_obj_trace_base_x4 <= gp0_obj_stage_base_x;
                        gp0_obj_trace_base_y4 <= gp0_obj_stage_base_y;
                        gp0_obj_trace_draw_x4 <= gp0_obj_stage_draw_x;
                        gp0_obj_trace_draw_y4 <= gp0_obj_stage_draw_y;
                    end
                    8'd5: begin
                        gp0_obj_trace_attr5 <= gp0_obj_word0;
                        gp0_obj_trace_code5 <= gp0_obj_word1;
                        gp0_obj_trace_base_x5 <= gp0_obj_stage_base_x;
                        gp0_obj_trace_base_y5 <= gp0_obj_stage_base_y;
                        gp0_obj_trace_draw_x5 <= gp0_obj_stage_draw_x;
                        gp0_obj_trace_draw_y5 <= gp0_obj_stage_draw_y;
                    end
                    default: begin
                    end
                endcase
`endif

                if (gp0_obj_stage_live) begin
                    gp0_obj_old_x <= gp0_obj_stage_base_x;
                    gp0_obj_old_y <= gp0_obj_stage_base_y;
                    if (gp0_obj_stage_store) begin
                        if (gp0_obj_count < OBJ_AUX_COUNT) begin
                            gp0_obj_x[gp0_obj_count] <= gp0_obj_stage_draw_x;
                            gp0_obj_y[gp0_obj_count] <= gp0_obj_stage_draw_y;
                            gp0_obj_raw_base_x[gp0_obj_count] <= gp0_obj_stage_base_x;
                            gp0_obj_raw_base_y[gp0_obj_count] <= gp0_obj_stage_base_y;
                            gp0_obj_w[gp0_obj_count] <= gp0_obj_stage_w;
                            gp0_obj_h[gp0_obj_count] <= gp0_obj_stage_h;
                            gp0_obj_attr[gp0_obj_count] <= gp0_obj_word0;
                            gp0_obj_code[gp0_obj_count] <=
                                {gp0_obj_word0[1:0], gp0_obj_word1};
                            gp0_obj_raw_x[gp0_obj_count] <= gp0_obj_word2;
                            gp0_obj_raw_y[gp0_obj_count] <= gp0_obj_word3;
                        end
                        gp0_obj_count <= gp0_obj_count + 7'd1;
                    end
                end

                if (gp1_obj_stage_live) begin
                    gp1_obj_old_x <= gp1_obj_stage_base_x;
                    gp1_obj_old_y <= gp1_obj_stage_base_y;
                    if (gp1_obj_stage_store) begin
                        if (gp1_obj_count < OBJ_AUX_COUNT) begin
                            gp1_obj_x[gp1_obj_count] <= gp1_obj_stage_draw_x;
                            gp1_obj_y[gp1_obj_count] <= gp1_obj_stage_draw_y;
                            gp1_obj_raw_base_x[gp1_obj_count] <= gp1_obj_stage_base_x;
                            gp1_obj_raw_base_y[gp1_obj_count] <= gp1_obj_stage_base_y;
                            gp1_obj_w[gp1_obj_count] <= gp1_obj_stage_w;
                            gp1_obj_h[gp1_obj_count] <= gp1_obj_stage_h;
                            gp1_obj_attr[gp1_obj_count] <= gp1_obj_word0;
                            gp1_obj_code[gp1_obj_count] <=
                                {gp1_obj_word0[1:0], gp1_obj_word1};
                            gp1_obj_raw_x[gp1_obj_count] <= gp1_obj_word2;
                            gp1_obj_raw_y[gp1_obj_count] <= gp1_obj_word3;
                        end
                        gp1_obj_count <= gp1_obj_count + 7'd1;
                    end
                end
                gp0_obj_stage_live <= 1'b0;
                gp1_obj_stage_live <= 1'b0;

                if (obj_scan_idx == 8'hff) begin
                    gp0_obj_live_latched <= gp0_obj_live_total_next;
                    gp1_obj_live_latched <= gp1_obj_live_total_next;
                    gp0_obj_visible_latched <= gp0_obj_visible_total_next;
                    gp1_obj_visible_latched <= gp1_obj_visible_total_next;
                    gp0_obj_cached_latched <= gp0_obj_cached_next;
                    gp1_obj_cached_latched <= gp1_obj_cached_next;
                    gp0_obj_visible_overflow_latched <= gp0_obj_visible_overflow_total_next;
                    gp1_obj_visible_overflow_latched <= gp1_obj_visible_overflow_total_next;
                    obj_cache_scan_active <= 1'b0;
                    obj_scan_phase <= 4'd10;
                end else begin
                    obj_scan_idx <= obj_scan_idx + 8'd1;
                    obj_scan_phase <= 4'd0;
                end
            end
            default: begin
            end
        endcase
end
end

always @(posedge clk) begin
    if (video_runtime_reset || !obj_linebuffer_render_en) begin
        obj_lb_state <= OBJ_LB_IDLE;
        obj_lb_target_y <= 9'd0;
        obj_lb_build_bank <= {OBJ_LB_BANK_BITS{1'b0}};
        obj_lb_frame_active <= 1'b0;
        obj_lb_cache_scan_active_d <= 1'b0;
        obj_lb_gp_sel <= 1'b0;
        obj_lb_slot <= 7'd0;
        obj_lb_clear_x <= 9'd0;
        obj_lb_obj_x <= 9'd0;
        obj_lb_obj_w <= 8'd0;
        obj_lb_obj_h <= 8'd0;
        obj_lb_obj_attr <= 16'h0000;
        obj_lb_obj_code <= 18'h00000;
        obj_lb_w_tiles <= 5'd0;
        obj_lb_tile_x <= 5'd0;
        obj_lb_tile_y <= 5'd0;
        obj_lb_row <= 3'd0;
        obj_lb_pixel <= 3'd0;
        obj_lb_tile_lo <= 16'h0000;
        obj_lb_tile_hi <= 16'h0000;
        obj_lb_req_addr <= {AW{1'b0}};
        obj_lb_req_high_addr <= {AW{1'b0}};
        obj_lb_req_pending <= 1'b0;
        obj_lb_req_hold <= 1'b0;
        obj_lb_bank_ready <= {OBJ_LB_BANK_COUNT{1'b0}};
        obj_lb_build_epoch <= 4'd0;
        if (!obj_lb_epochs_initialized) begin
            obj_lb_epochs_initialized <= 1'b1;
            for (obj_lb_bank_i = 0; obj_lb_bank_i < OBJ_LB_BANK_COUNT;
                 obj_lb_bank_i = obj_lb_bank_i + 1) begin
                obj_lb_bank_epoch[obj_lb_bank_i] <= 4'd0;
            end
        end
        obj_lb_line_cycles <= 13'd0;
        obj_lb_last_cycles <= 13'd0;
        obj_lb_max_cycles <= 13'd0;
        obj_lb_deadline_miss_count <= 8'd0;
        obj_lb_max_cycles_latched <= 13'd0;
        obj_lb_deadline_miss_latched <= 8'd0;
    end else begin
        obj_lb_cache_scan_active_d <= obj_cache_scan_active;

        // Stop using the old object snapshot as soon as the vblank scan
        // begins. Once it completes, fill a circular line queue as
        // quickly as tile-priority bandwidth permits.
        if (obj_lb_cache_scan_start) begin
            obj_lb_max_cycles_latched <= obj_lb_max_cycles;
            obj_lb_deadline_miss_latched <= obj_lb_deadline_miss_count;
            obj_lb_req_pending <= 1'b0;
            obj_lb_req_hold <= 1'b0;
            obj_lb_frame_active <= 1'b0;
            obj_lb_state <= OBJ_LB_IDLE;
        end else if (obj_lb_cache_scan_done) begin
            obj_lb_target_y <= 9'd0;
            obj_lb_build_bank <= {OBJ_LB_BANK_BITS{1'b0}};
            obj_lb_frame_active <= 1'b1;
            obj_lb_bank_ready <= {OBJ_LB_BANK_COUNT{1'b0}};
            obj_lb_gp_sel <= 1'b0;
            obj_lb_slot <= 7'd0;
            obj_lb_clear_x <= 9'd0;
            obj_lb_req_pending <= 1'b0;
            obj_lb_req_hold <= 1'b0;
            obj_lb_line_cycles <= 13'd0;
            obj_lb_last_cycles <= 13'd0;
            obj_lb_max_cycles <= 13'd0;
            obj_lb_deadline_miss_count <= 8'd0;
            obj_lb_state <= OBJ_LB_IDLE;
        end else if (obj_lb_active_line_late) begin
            if (obj_lb_deadline_miss_event &&
                (obj_lb_deadline_miss_count != 8'hff)) begin
                obj_lb_deadline_miss_count <=
                    obj_lb_deadline_miss_count + 8'd1;
            end
            obj_lb_req_pending <= 1'b0;
            obj_lb_req_hold <= 1'b0;
            obj_lb_line_cycles <= 13'd0;
            obj_lb_state <= OBJ_LB_IDLE;
            if (vcnt < (V_END - 9'd1)) begin
                obj_lb_target_y <= vcnt + 9'd1;
                obj_lb_frame_active <= 1'b1;
            end else begin
                obj_lb_frame_active <= 1'b0;
            end
        end else begin
            if (obj_lb_req_pending && !obj_lb_req_hold &&
                obj_lb_slot_start_safe) begin
                obj_lb_req_hold <= 1'b1;
            end
            if (obj_lb_deadline_miss_event &&
                (obj_lb_deadline_miss_count != 8'hff)) begin
                obj_lb_deadline_miss_count <=
                    obj_lb_deadline_miss_count + 8'd1;
            end
            if ((obj_lb_state != OBJ_LB_IDLE) &&
                (obj_lb_line_cycles != 13'h1fff)) begin
                obj_lb_line_cycles <= obj_lb_line_cycles + 13'd1;
            end

            case (obj_lb_state)
                OBJ_LB_IDLE: begin
                    if (obj_lb_can_start_line) begin
                        obj_lb_build_bank <=
                            obj_lb_target_y[OBJ_LB_BANK_BITS-1:0];
                        obj_lb_bank_ready[
                            obj_lb_target_y[OBJ_LB_BANK_BITS-1:0]] <= 1'b0;
                        obj_lb_build_epoch <= obj_lb_target_epoch_next;
                        obj_lb_bank_epoch[
                            obj_lb_target_y[OBJ_LB_BANK_BITS-1:0]] <=
                            obj_lb_target_epoch_next;
                        obj_lb_gp_sel <= 1'b0;
                        obj_lb_slot <= 7'd0;
                        obj_lb_clear_x <= 9'd0;
                        obj_lb_req_pending <= 1'b0;
                        obj_lb_req_hold <= 1'b0;
                        obj_lb_line_cycles <= 13'd0;
                        obj_lb_state <= (obj_lb_target_epoch_next == 4'd0) ?
                                        OBJ_LB_CLEAR : OBJ_LB_CACHE_WAIT;
                    end
                end

                OBJ_LB_CLEAR: begin
                    if (obj_lb_clear_x == (H_END - 10'd1)) begin
                        obj_lb_clear_x <= 9'd0;
                        obj_lb_gp_sel <= 1'b0;
                        obj_lb_slot <= 7'd0;
                        obj_lb_state <= OBJ_LB_CACHE_WAIT;
                    end else begin
                        obj_lb_clear_x <= obj_lb_clear_x + 9'd1;
                    end
                end

                OBJ_LB_CACHE_WAIT: begin
                    obj_lb_state <= OBJ_LB_CACHE_READ;
                end

                // The cache M10Ks register both their address and output.
                // Keep the slot stable for both cycles before consuming q.
                OBJ_LB_CACHE_READ: begin
                    obj_lb_state <= OBJ_LB_OBJECT;
                end

                OBJ_LB_OBJECT: begin
                    if (obj_lb_slot >= obj_lb_selected_count) begin
                        if (!obj_lb_gp_sel) begin
                            obj_lb_gp_sel <= 1'b1;
                            obj_lb_slot <= 7'd0;
                            obj_lb_state <= OBJ_LB_CACHE_WAIT;
                        end else begin
                            obj_lb_bank_ready[obj_lb_build_bank] <= 1'b1;
                            obj_lb_bank_y[obj_lb_build_bank] <= obj_lb_target_y;
                            obj_lb_last_cycles <= obj_lb_line_cycles;
                            if (obj_lb_line_cycles > obj_lb_max_cycles) begin
                                obj_lb_max_cycles <= obj_lb_line_cycles;
                            end
                            if (obj_lb_target_y == (V_END - 9'd1)) begin
                                obj_lb_frame_active <= 1'b0;
                            end else begin
                                obj_lb_target_y <= obj_lb_target_y + 9'd1;
                            end
                            obj_lb_state <= OBJ_LB_IDLE;
                        end
                    end else if (obj_lb_selected_on_line) begin
                        obj_lb_obj_x <= obj_lb_selected_x;
                        obj_lb_obj_w <= obj_lb_selected_w;
                        obj_lb_obj_h <= obj_lb_selected_h;
                        obj_lb_obj_attr <= obj_lb_selected_attr;
                        obj_lb_obj_code <= obj_lb_selected_code;
                        obj_lb_w_tiles <= obj_lb_selected_w_tiles;
                        obj_lb_tile_x <= 5'd0;
                        obj_lb_tile_y <= obj_lb_selected_tile_y;
                        obj_lb_row <= obj_lb_selected_row;
                        obj_lb_state <= OBJ_LB_TILE;
                    end else begin
                        obj_lb_slot <= obj_lb_slot + 7'd1;
                        obj_lb_state <= OBJ_LB_CACHE_WAIT;
                    end
                end

                OBJ_LB_TILE: begin
                    if (obj_lb_tile_x >= obj_lb_w_tiles) begin
                        obj_lb_slot <= obj_lb_slot + 7'd1;
                        obj_lb_state <= OBJ_LB_CACHE_WAIT;
                    end else if (obj_lb_tile_visible) begin
                        obj_lb_req_addr <= obj_lb_tile_addr_lo;
                        obj_lb_req_high_addr <= obj_lb_tile_addr_hi;
                        obj_lb_req_pending <= 1'b1;
                        obj_lb_state <= OBJ_LB_FETCH_LO;
                    end else begin
                        obj_lb_tile_x <= obj_lb_tile_x + 5'd1;
                    end
                end

                OBJ_LB_FETCH_LO: begin
                    if (obj_lb_req_pending && obj_lb_slot_grant &&
                        gfx_obj_slot_ok) begin
                        obj_lb_state <= OBJ_LB_LATCH_LO;
                    end
                end

                OBJ_LB_LATCH_LO: begin
                    obj_lb_tile_lo <= gfx_obj_slot_dout;
                    obj_lb_req_addr <= obj_lb_req_high_addr;
                    obj_lb_state <= OBJ_LB_FETCH_HI;
                end

                OBJ_LB_FETCH_HI: begin
                    if (obj_lb_req_pending && obj_lb_slot_grant &&
                        gfx_obj_slot_ok) begin
                        obj_lb_state <= OBJ_LB_LATCH_HI;
                    end
                end

                OBJ_LB_LATCH_HI: begin
                    obj_lb_tile_hi <= gfx_obj_slot_dout;
                    obj_lb_req_pending <= 1'b0;
                    obj_lb_req_hold <= 1'b0;
                    obj_lb_pixel <= 3'd0;
                    obj_lb_state <= OBJ_LB_PIXEL_READ;
                end

                OBJ_LB_PIXEL_READ: begin
                    if ((obj_lb_pixel_x >= H_END[8:0]) ||
                        (obj_lb_pixel_pen == 4'h0)) begin
                        if (obj_lb_pixel == 3'd7) begin
                            obj_lb_tile_x <= obj_lb_tile_x + 5'd1;
                            obj_lb_state <= OBJ_LB_TILE;
                        end else begin
                            obj_lb_pixel <= obj_lb_pixel + 3'd1;
                        end
                    end else begin
                        obj_lb_state <= OBJ_LB_PIXEL_WRITE;
                    end
                end

                OBJ_LB_PIXEL_WRITE: begin
                    if (obj_lb_pixel == 3'd7) begin
                        obj_lb_tile_x <= obj_lb_tile_x + 5'd1;
                        obj_lb_state <= OBJ_LB_TILE;
                    end else begin
                        obj_lb_pixel <= obj_lb_pixel + 3'd1;
                        obj_lb_state <= OBJ_LB_PIXEL_READ;
                    end
                end

                default: begin
                    obj_lb_state <= OBJ_LB_IDLE;
                    obj_lb_req_pending <= 1'b0;
                    obj_lb_req_hold <= 1'b0;
                end
            endcase
        end
    end
end

always @(posedge clk) begin
    if (video_runtime_reset || gp_tile_hex_mode ||
        !obj_line_render_en) begin
        obj_line_req_addr <= {AW{1'b0}};
        obj_line_req_high_addr <= {AW{1'b0}};
        obj_line_req_next_addr <= {AW{1'b0}};
        obj_line_req_next_high_addr <= {AW{1'b0}};
        obj_line_req_alt_addr <= {AW{1'b0}};
        obj_line_req_alt_high_addr <= {AW{1'b0}};
        obj_line_req_alt_next_addr <= {AW{1'b0}};
        obj_line_req_alt_next_high_addr <= {AW{1'b0}};
        obj_line_req_pending <= 1'b0;
        obj_line_req_phase <= 3'd0;
        obj_line_req_valid <= 1'b0;
        obj_line_req_gp_sel <= 1'b0;
        obj_line_req_flipx <= 1'b0;
        obj_line_req_next_valid <= 1'b0;
        obj_line_req_mask <= 8'h00;
        obj_line_req_bias <= 4'h0;
        obj_line_req_color <= 7'h00;
        obj_line_req_pri <= 4'h0;
        obj_line_req_alt_valid <= 1'b0;
        obj_line_req_alt_gp_sel <= 1'b0;
        obj_line_req_alt_flipx <= 1'b0;
        obj_line_req_alt_next_valid <= 1'b0;
        obj_line_req_alt_mask <= 8'h00;
        obj_line_req_alt_bias <= 4'h0;
        obj_line_req_alt_color <= 7'h00;
        obj_line_req_alt_pri <= 4'h0;
        obj_line_req_target_x <= 9'd0;
        obj_line_req_target_y <= 9'd0;
        obj_line_req_primary_committed <= 1'b0;
        obj_line_req_commit_slot <= OBJ_LINE_COMMIT_NONE;
        obj_line_fetch_lo <= 16'h0000;
        obj_line_tile0_lo <= 16'h0000;
        obj_line_tile0_hi <= 16'h0000;
        obj_line_primary_lo <= 16'h0000;
        obj_line_primary_hi <= 16'h0000;
        obj_line_alt_lo <= 16'h0000;
        obj_line_alt_hi <= 16'h0000;
        obj_line_primary_commit_pending <= 1'b0;
        obj_line_alt_commit_pending <= 1'b0;
        obj_line_prefetch_lo <= 16'h0000;
        obj_line_prefetch_hi <= 16'h0000;
        obj_line_prefetch_alt_lo <= 16'h0000;
        obj_line_prefetch_alt_hi <= 16'h0000;
        obj_line_prefetch_color <= 7'h00;
        obj_line_prefetch_pri <= 4'h0;
        obj_line_prefetch_alt_color <= 7'h00;
        obj_line_prefetch_alt_pri <= 4'h0;
        obj_line_prefetch_x <= 9'd0;
        obj_line_prefetch_y <= 9'd0;
        obj_line_prefetch_valid <= 1'b0;
        obj_line_prefetch_alt_valid <= 1'b0;
        obj_line_prefetch_ready <= 1'b0;
        obj_line_prefetch_gp_sel <= 1'b0;
        obj_line_prefetch_flipx <= 1'b0;
        obj_line_prefetch_alt_gp_sel <= 1'b0;
        obj_line_prefetch_alt_flipx <= 1'b0;
        obj_line_prefetch1_lo <= 16'h0000;
        obj_line_prefetch1_hi <= 16'h0000;
        obj_line_prefetch1_alt_lo <= 16'h0000;
        obj_line_prefetch1_alt_hi <= 16'h0000;
        obj_line_prefetch1_color <= 7'h00;
        obj_line_prefetch1_pri <= 4'h0;
        obj_line_prefetch1_alt_color <= 7'h00;
        obj_line_prefetch1_alt_pri <= 4'h0;
        obj_line_prefetch1_x <= 9'd0;
        obj_line_prefetch1_y <= 9'd0;
        obj_line_prefetch1_valid <= 1'b0;
        obj_line_prefetch1_alt_valid <= 1'b0;
        obj_line_prefetch1_ready <= 1'b0;
        obj_line_prefetch1_gp_sel <= 1'b0;
        obj_line_prefetch1_flipx <= 1'b0;
        obj_line_prefetch1_alt_gp_sel <= 1'b0;
        obj_line_prefetch1_alt_flipx <= 1'b0;
        obj_line_prefetch2_lo <= 16'h0000;
        obj_line_prefetch2_hi <= 16'h0000;
        obj_line_prefetch2_alt_lo <= 16'h0000;
        obj_line_prefetch2_alt_hi <= 16'h0000;
        obj_line_prefetch2_color <= 7'h00;
        obj_line_prefetch2_pri <= 4'h0;
        obj_line_prefetch2_alt_color <= 7'h00;
        obj_line_prefetch2_alt_pri <= 4'h0;
        obj_line_prefetch2_x <= 9'd0;
        obj_line_prefetch2_y <= 9'd0;
        obj_line_prefetch2_valid <= 1'b0;
        obj_line_prefetch2_alt_valid <= 1'b0;
        obj_line_prefetch2_ready <= 1'b0;
        obj_line_prefetch2_gp_sel <= 1'b0;
        obj_line_prefetch2_flipx <= 1'b0;
        obj_line_prefetch2_alt_gp_sel <= 1'b0;
        obj_line_prefetch2_alt_flipx <= 1'b0;
        for (obj_line_extra_seq_i = 0;
             obj_line_extra_seq_i < OBJ_LINE_EXTRA_PREFETCH_SLOTS;
             obj_line_extra_seq_i = obj_line_extra_seq_i + 1) begin
            obj_line_prefetch_extra_lo[obj_line_extra_seq_i] <= 16'h0000;
            obj_line_prefetch_extra_hi[obj_line_extra_seq_i] <= 16'h0000;
            obj_line_prefetch_extra_alt_lo[obj_line_extra_seq_i] <= 16'h0000;
            obj_line_prefetch_extra_alt_hi[obj_line_extra_seq_i] <= 16'h0000;
            obj_line_prefetch_extra_color[obj_line_extra_seq_i] <= 7'h00;
            obj_line_prefetch_extra_pri[obj_line_extra_seq_i] <= 4'h0;
            obj_line_prefetch_extra_alt_color[obj_line_extra_seq_i] <= 7'h00;
            obj_line_prefetch_extra_alt_pri[obj_line_extra_seq_i] <= 4'h0;
            obj_line_prefetch_extra_x[obj_line_extra_seq_i] <= 9'd0;
            obj_line_prefetch_extra_y[obj_line_extra_seq_i] <= 9'd0;
            obj_line_prefetch_extra_valid[obj_line_extra_seq_i] <= 1'b0;
            obj_line_prefetch_extra_alt_valid[obj_line_extra_seq_i] <= 1'b0;
            obj_line_prefetch_extra_ready[obj_line_extra_seq_i] <= 1'b0;
            obj_line_prefetch_extra_gp_sel[obj_line_extra_seq_i] <= 1'b0;
            obj_line_prefetch_extra_flipx[obj_line_extra_seq_i] <= 1'b0;
            obj_line_prefetch_extra_alt_gp_sel[obj_line_extra_seq_i] <= 1'b0;
            obj_line_prefetch_extra_alt_flipx[obj_line_extra_seq_i] <= 1'b0;
        end
        obj_line_latched_lo <= 16'h0000;
        obj_line_latched_hi <= 16'h0000;
        obj_line_latched_alt_lo <= 16'h0000;
        obj_line_latched_alt_hi <= 16'h0000;
        obj_line_latched_color <= 7'h00;
        obj_line_latched_pri <= 4'h0;
        obj_line_latched_alt_color <= 7'h00;
        obj_line_latched_alt_pri <= 4'h0;
        obj_line_latched_valid <= 1'b0;
        obj_line_latched_alt_valid <= 1'b0;
        obj_line_latched_gp_sel <= 1'b0;
        obj_line_latched_flipx <= 1'b0;
        obj_line_latched_alt_gp_sel <= 1'b0;
        obj_line_latched_alt_flipx <= 1'b0;
        obj_line_seen_ok <= 1'b0;
        obj_line_seen_nonzero <= 1'b0;
        obj_seq_scan_active <= 1'b0;
        obj_seq_scan_idx <= 6'd0;
        obj_seq_target_x <= 9'd0;
        obj_seq_target_y <= 9'd0;
        obj_seq_pick_ready <= 1'b0;
        obj_seq_pick_valid <= 1'b0;
        obj_seq_pick_gp_sel <= 1'b0;
        obj_seq_pick_code <= 18'h00000;
        obj_seq_pick_next_code <= 18'h00000;
        obj_seq_pick_next_valid <= 1'b0;
        obj_seq_pick_mask <= 8'h00;
        obj_seq_pick_bias <= 4'h0;
        obj_seq_pick_row <= 3'd0;
        obj_seq_pick_color <= 7'h00;
        obj_seq_pick_pri <= 4'h0;
        obj_seq_pick_flipx <= 1'b0;
        obj_seq_pick2_valid <= 1'b0;
        obj_seq_pick2_gp_sel <= 1'b0;
        obj_seq_pick2_code <= 18'h00000;
        obj_seq_pick2_next_code <= 18'h00000;
        obj_seq_pick2_next_valid <= 1'b0;
        obj_seq_pick2_mask <= 8'h00;
        obj_seq_pick2_bias <= 4'h0;
        obj_seq_pick2_row <= 3'd0;
        obj_seq_pick2_color <= 7'h00;
        obj_seq_pick2_pri <= 4'h0;
        obj_seq_pick2_flipx <= 1'b0;
        obj_seq_pick_target_x <= 9'd0;
        obj_seq_pick_target_y <= 9'd0;
        obj_seq_eval_valid <= 1'b0;
        obj_seq_eval_gp_sel <= 1'b0;
        obj_seq_eval_attr <= 16'h0000;
        obj_seq_eval_code_base <= 18'h00000;
        obj_seq_eval_dx <= 9'd0;
        obj_seq_eval_dy <= 9'd0;
        obj_seq_eval_w_tiles <= 5'd0;
        obj_seq_eval_h_tiles <= 5'd0;
        obj_seq_scan_done_pending <= 1'b0;
    end else begin
        if (debug_frame_tick && obj_bank_diag_en) begin
            obj_line_req_pending <= 1'b0;
            obj_line_req_phase <= 3'd0;
            obj_line_req_valid <= 1'b0;
            obj_line_req_alt_valid <= 1'b0;
            obj_line_req_primary_committed <= 1'b0;
            obj_line_req_commit_slot <= OBJ_LINE_COMMIT_NONE;
            obj_line_primary_commit_pending <= 1'b0;
            obj_line_alt_commit_pending <= 1'b0;
            obj_line_prefetch_valid <= 1'b0;
            obj_line_prefetch_alt_valid <= 1'b0;
            obj_line_prefetch_ready <= 1'b0;
            obj_line_prefetch1_valid <= 1'b0;
            obj_line_prefetch1_alt_valid <= 1'b0;
            obj_line_prefetch1_ready <= 1'b0;
            obj_line_prefetch2_valid <= 1'b0;
            obj_line_prefetch2_alt_valid <= 1'b0;
            obj_line_prefetch2_ready <= 1'b0;
            for (obj_line_extra_seq_i = 0;
                 obj_line_extra_seq_i < OBJ_LINE_EXTRA_PREFETCH_SLOTS;
                 obj_line_extra_seq_i = obj_line_extra_seq_i + 1) begin
                obj_line_prefetch_extra_valid[obj_line_extra_seq_i] <= 1'b0;
                obj_line_prefetch_extra_alt_valid[obj_line_extra_seq_i] <= 1'b0;
                obj_line_prefetch_extra_ready[obj_line_extra_seq_i] <= 1'b0;
            end
            obj_line_latched_valid <= 1'b0;
            obj_line_latched_alt_valid <= 1'b0;
            obj_seq_scan_active <= 1'b0;
            obj_seq_pick_ready <= 1'b0;
            obj_seq_pick_valid <= 1'b0;
            obj_seq_pick2_valid <= 1'b0;
            obj_seq_eval_valid <= 1'b0;
            obj_seq_scan_done_pending <= 1'b0;
        end else if ((clkdiv == 4'd1) && obj_line_sched_word_event) begin
            if (obj_line_word_event) begin
                if (obj_line_prefetch_match_event) begin
                obj_line_latched_lo <= obj_line_prefetch_lo;
                obj_line_latched_hi <= obj_line_prefetch_hi;
                obj_line_latched_alt_lo <= obj_line_prefetch_alt_lo;
                obj_line_latched_alt_hi <= obj_line_prefetch_alt_hi;
                obj_line_latched_color <= obj_line_prefetch_color;
                obj_line_latched_pri <= obj_line_prefetch_pri;
                obj_line_latched_alt_color <= obj_line_prefetch_alt_color;
                obj_line_latched_alt_pri <= obj_line_prefetch_alt_pri;
                obj_line_latched_gp_sel <= obj_line_prefetch_gp_sel;
                obj_line_latched_flipx <= obj_line_prefetch_flipx;
                obj_line_latched_alt_gp_sel <= obj_line_prefetch_alt_gp_sel;
                obj_line_latched_alt_flipx <= obj_line_prefetch_alt_flipx;
                obj_line_latched_valid <= 1'b1;
                obj_line_latched_alt_valid <= obj_line_prefetch_alt_valid;
                obj_line_prefetch_ready <= 1'b0;
                obj_line_prefetch_valid <= 1'b0;
                obj_line_prefetch_alt_valid <= 1'b0;
                end else if (obj_line_prefetch1_match_event) begin
                obj_line_latched_lo <= obj_line_prefetch1_lo;
                obj_line_latched_hi <= obj_line_prefetch1_hi;
                obj_line_latched_alt_lo <= obj_line_prefetch1_alt_lo;
                obj_line_latched_alt_hi <= obj_line_prefetch1_alt_hi;
                obj_line_latched_color <= obj_line_prefetch1_color;
                obj_line_latched_pri <= obj_line_prefetch1_pri;
                obj_line_latched_alt_color <= obj_line_prefetch1_alt_color;
                obj_line_latched_alt_pri <= obj_line_prefetch1_alt_pri;
                obj_line_latched_gp_sel <= obj_line_prefetch1_gp_sel;
                obj_line_latched_flipx <= obj_line_prefetch1_flipx;
                obj_line_latched_alt_gp_sel <= obj_line_prefetch1_alt_gp_sel;
                obj_line_latched_alt_flipx <= obj_line_prefetch1_alt_flipx;
                obj_line_latched_valid <= 1'b1;
                obj_line_latched_alt_valid <= obj_line_prefetch1_alt_valid;
                obj_line_prefetch1_ready <= 1'b0;
                obj_line_prefetch1_valid <= 1'b0;
                obj_line_prefetch1_alt_valid <= 1'b0;
                end else if (obj_line_prefetch2_match_event) begin
                obj_line_latched_lo <= obj_line_prefetch2_lo;
                obj_line_latched_hi <= obj_line_prefetch2_hi;
                obj_line_latched_alt_lo <= obj_line_prefetch2_alt_lo;
                obj_line_latched_alt_hi <= obj_line_prefetch2_alt_hi;
                obj_line_latched_color <= obj_line_prefetch2_color;
                obj_line_latched_pri <= obj_line_prefetch2_pri;
                obj_line_latched_alt_color <= obj_line_prefetch2_alt_color;
                obj_line_latched_alt_pri <= obj_line_prefetch2_alt_pri;
                obj_line_latched_gp_sel <= obj_line_prefetch2_gp_sel;
                obj_line_latched_flipx <= obj_line_prefetch2_flipx;
                obj_line_latched_alt_gp_sel <= obj_line_prefetch2_alt_gp_sel;
                obj_line_latched_alt_flipx <= obj_line_prefetch2_alt_flipx;
                obj_line_latched_valid <= 1'b1;
                obj_line_latched_alt_valid <= obj_line_prefetch2_alt_valid;
                obj_line_prefetch2_ready <= 1'b0;
                obj_line_prefetch2_valid <= 1'b0;
                obj_line_prefetch2_alt_valid <= 1'b0;
                end else if (obj_line_extra_match_event) begin
                obj_line_latched_lo <= obj_line_prefetch_extra_lo[obj_line_extra_match_idx_event];
                obj_line_latched_hi <= obj_line_prefetch_extra_hi[obj_line_extra_match_idx_event];
                obj_line_latched_alt_lo <= obj_line_prefetch_extra_alt_lo[obj_line_extra_match_idx_event];
                obj_line_latched_alt_hi <= obj_line_prefetch_extra_alt_hi[obj_line_extra_match_idx_event];
                obj_line_latched_color <= obj_line_prefetch_extra_color[obj_line_extra_match_idx_event];
                obj_line_latched_pri <= obj_line_prefetch_extra_pri[obj_line_extra_match_idx_event];
                obj_line_latched_alt_color <= obj_line_prefetch_extra_alt_color[obj_line_extra_match_idx_event];
                obj_line_latched_alt_pri <= obj_line_prefetch_extra_alt_pri[obj_line_extra_match_idx_event];
                obj_line_latched_gp_sel <= obj_line_prefetch_extra_gp_sel[obj_line_extra_match_idx_event];
                obj_line_latched_flipx <= obj_line_prefetch_extra_flipx[obj_line_extra_match_idx_event];
                obj_line_latched_alt_gp_sel <= obj_line_prefetch_extra_alt_gp_sel[obj_line_extra_match_idx_event];
                obj_line_latched_alt_flipx <= obj_line_prefetch_extra_alt_flipx[obj_line_extra_match_idx_event];
                obj_line_latched_valid <= 1'b1;
                obj_line_latched_alt_valid <= obj_line_prefetch_extra_alt_valid[obj_line_extra_match_idx_event];
                obj_line_prefetch_extra_ready[obj_line_extra_match_idx_event] <= 1'b0;
                obj_line_prefetch_extra_valid[obj_line_extra_match_idx_event] <= 1'b0;
                obj_line_prefetch_extra_alt_valid[obj_line_extra_match_idx_event] <= 1'b0;
                end else begin
                obj_line_latched_valid <= 1'b0;
                obj_line_latched_alt_valid <= 1'b0;
                if (obj_line_prefetch_expired_event) begin
                    obj_line_prefetch_ready <= 1'b0;
                    obj_line_prefetch_valid <= 1'b0;
                    obj_line_prefetch_alt_valid <= 1'b0;
                end
                if (obj_line_prefetch1_expired_event) begin
                    obj_line_prefetch1_ready <= 1'b0;
                    obj_line_prefetch1_valid <= 1'b0;
                    obj_line_prefetch1_alt_valid <= 1'b0;
                end
                if (obj_line_prefetch2_expired_event) begin
                    obj_line_prefetch2_ready <= 1'b0;
                    obj_line_prefetch2_valid <= 1'b0;
                    obj_line_prefetch2_alt_valid <= 1'b0;
                end
                for (obj_line_extra_seq_i = 0;
                     obj_line_extra_seq_i < OBJ_LINE_EXTRA_PREFETCH_SLOTS;
                     obj_line_extra_seq_i = obj_line_extra_seq_i + 1) begin
                    if (obj_line_extra_expired_event[obj_line_extra_seq_i]) begin
                        obj_line_prefetch_extra_ready[obj_line_extra_seq_i] <= 1'b0;
                        obj_line_prefetch_extra_valid[obj_line_extra_seq_i] <= 1'b0;
                        obj_line_prefetch_extra_alt_valid[obj_line_extra_seq_i] <= 1'b0;
                    end
                end
                end
            end

            if (obj_seq_pick_launchable) begin
                obj_line_req_addr <= obj_line_pick_addr_lo;
                obj_line_req_high_addr <= obj_line_pick_addr_hi;
                obj_line_req_next_addr <= obj_line_pick_next_addr_lo;
                obj_line_req_next_high_addr <= obj_line_pick_next_addr_hi;
                obj_line_req_alt_addr <= obj_line_pick2_addr_lo;
                obj_line_req_alt_high_addr <= obj_line_pick2_addr_hi;
                obj_line_req_alt_next_addr <= obj_line_pick2_next_addr_lo;
                obj_line_req_alt_next_high_addr <= obj_line_pick2_next_addr_hi;
                obj_line_req_valid <= 1'b1;
                obj_line_req_gp_sel <= obj_seq_pick_gp_sel;
                obj_line_req_flipx <= obj_seq_pick_flipx;
                obj_line_req_next_valid <= obj_seq_pick_next_valid;
                obj_line_req_mask <= obj_seq_pick_mask;
                obj_line_req_bias <= obj_seq_pick_bias;
                obj_line_req_color <= obj_seq_pick_color;
                obj_line_req_pri <= obj_seq_pick_pri;
                obj_line_req_alt_valid <= OBJ_LINE_SECONDARY_FETCH && obj_seq_pick2_valid;
                obj_line_req_alt_gp_sel <= obj_seq_pick2_gp_sel;
                obj_line_req_alt_flipx <= obj_seq_pick2_flipx;
                obj_line_req_alt_next_valid <= obj_seq_pick2_next_valid;
                obj_line_req_alt_mask <= obj_seq_pick2_mask;
                obj_line_req_alt_bias <= obj_seq_pick2_bias;
                obj_line_req_alt_color <= obj_seq_pick2_color;
                obj_line_req_alt_pri <= obj_seq_pick2_pri;
                obj_line_req_target_x <= obj_seq_pick_target_x;
                obj_line_req_target_y <= obj_seq_pick_target_y;
                obj_line_req_pending <= 1'b1;
                obj_line_req_phase <= 3'd0;
                obj_line_req_primary_committed <= 1'b0;
                obj_line_req_commit_slot <= OBJ_LINE_COMMIT_NONE;
                obj_line_primary_commit_pending <= 1'b0;
                obj_line_alt_commit_pending <= 1'b0;

                obj_seq_scan_active <= obj_seq_after_launch_visible;
                obj_seq_scan_idx <= 6'd0;
                obj_seq_target_x <= obj_seq_after_launch_target_x;
                obj_seq_target_y <= obj_seq_after_launch_target_y;
                obj_seq_pick_ready <= 1'b0;
                obj_seq_pick_valid <= 1'b0;
                obj_seq_pick_gp_sel <= 1'b0;
                obj_seq_pick_code <= 18'h00000;
                obj_seq_pick_next_code <= 18'h00000;
                obj_seq_pick_next_valid <= 1'b0;
                obj_seq_pick_mask <= 8'h00;
                obj_seq_pick_bias <= 4'h0;
                obj_seq_pick_row <= 3'd0;
                obj_seq_pick_color <= 7'h00;
                obj_seq_pick_pri <= 4'h0;
                obj_seq_pick_flipx <= 1'b0;
                obj_seq_pick2_valid <= 1'b0;
                obj_seq_pick2_gp_sel <= 1'b0;
                obj_seq_pick2_code <= 18'h00000;
                obj_seq_pick2_next_code <= 18'h00000;
                obj_seq_pick2_next_valid <= 1'b0;
                obj_seq_pick2_mask <= 8'h00;
                obj_seq_pick2_bias <= 4'h0;
                obj_seq_pick2_row <= 3'd0;
                obj_seq_pick2_color <= 7'h00;
                obj_seq_pick2_pri <= 4'h0;
                obj_seq_pick2_flipx <= 1'b0;
                obj_seq_pick_target_x <= obj_seq_after_launch_target_x;
                obj_seq_pick_target_y <= obj_seq_after_launch_target_y;
                obj_seq_eval_valid <= 1'b0;
                obj_seq_eval_gp_sel <= 1'b0;
                obj_seq_eval_attr <= 16'h0000;
                obj_seq_eval_code_base <= 18'h00000;
                obj_seq_eval_dx <= 9'd0;
                obj_seq_eval_dy <= 9'd0;
                obj_seq_eval_w_tiles <= 5'd0;
                obj_seq_eval_h_tiles <= 5'd0;
                obj_seq_scan_done_pending <= 1'b0;
            end else if (!obj_seq_pick_ready || !obj_seq_pick_valid) begin
                obj_seq_scan_active <= obj_seq_restart_target_visible;
                obj_seq_scan_idx <= 6'd0;
                obj_seq_target_x <= obj_seq_restart_target_x;
                obj_seq_target_y <= obj_seq_restart_target_y;
                obj_seq_pick_ready <= 1'b0;
                obj_seq_pick_valid <= 1'b0;
                obj_seq_pick_gp_sel <= 1'b0;
                obj_seq_pick_code <= 18'h00000;
                obj_seq_pick_next_code <= 18'h00000;
                obj_seq_pick_next_valid <= 1'b0;
                obj_seq_pick_mask <= 8'h00;
                obj_seq_pick_bias <= 4'h0;
                obj_seq_pick_row <= 3'd0;
                obj_seq_pick_color <= 7'h00;
                obj_seq_pick_pri <= 4'h0;
                obj_seq_pick_flipx <= 1'b0;
                obj_seq_pick2_valid <= 1'b0;
                obj_seq_pick2_gp_sel <= 1'b0;
                obj_seq_pick2_code <= 18'h00000;
                obj_seq_pick2_next_code <= 18'h00000;
                obj_seq_pick2_next_valid <= 1'b0;
                obj_seq_pick2_mask <= 8'h00;
                obj_seq_pick2_bias <= 4'h0;
                obj_seq_pick2_row <= 3'd0;
                obj_seq_pick2_color <= 7'h00;
                obj_seq_pick2_pri <= 4'h0;
                obj_seq_pick2_flipx <= 1'b0;
                obj_seq_pick_target_x <= obj_seq_restart_target_x;
                obj_seq_pick_target_y <= obj_seq_restart_target_y;
                obj_seq_eval_valid <= 1'b0;
                obj_seq_eval_gp_sel <= 1'b0;
                obj_seq_eval_attr <= 16'h0000;
                obj_seq_eval_code_base <= 18'h00000;
                obj_seq_eval_dx <= 9'd0;
                obj_seq_eval_dy <= 9'd0;
                obj_seq_eval_w_tiles <= 5'd0;
                obj_seq_eval_h_tiles <= 5'd0;
                obj_seq_scan_done_pending <= 1'b0;
            end
        end else if (obj_seq_scan_active) begin
            if (obj_seq_eval_wins_primary) begin
                if (obj_seq_pick_valid) begin
                    obj_seq_pick2_valid <= 1'b1;
                    obj_seq_pick2_gp_sel <= obj_seq_pick_gp_sel;
                    obj_seq_pick2_code <= obj_seq_pick_code;
                    obj_seq_pick2_next_code <= obj_seq_pick_next_code;
                    obj_seq_pick2_next_valid <= obj_seq_pick_next_valid;
                    obj_seq_pick2_mask <= obj_seq_pick_mask;
                    obj_seq_pick2_bias <= obj_seq_pick_bias;
                    obj_seq_pick2_row <= obj_seq_pick_row;
                    obj_seq_pick2_color <= obj_seq_pick_color;
                    obj_seq_pick2_pri <= obj_seq_pick_pri;
                    obj_seq_pick2_flipx <= obj_seq_pick_flipx;
                end
                obj_seq_pick_valid <= 1'b1;
                obj_seq_pick_gp_sel <= obj_seq_eval_gp_sel;
                obj_seq_pick_code <= obj_seq_eval_code;
                obj_seq_pick_next_code <= obj_seq_eval_next_code;
                obj_seq_pick_next_valid <= obj_seq_eval_next_valid;
                obj_seq_pick_mask <= obj_seq_eval_word_mask;
                obj_seq_pick_bias <= obj_seq_eval_source_bias;
                obj_seq_pick_row <= obj_seq_eval_row;
                obj_seq_pick_color <= obj_seq_eval_attr[7:2];
                obj_seq_pick_pri <= obj_seq_eval_attr[11:8];
                obj_seq_pick_flipx <= obj_seq_eval_attr[12];
            end else if (obj_seq_eval_wins_secondary) begin
                obj_seq_pick2_valid <= 1'b1;
                obj_seq_pick2_gp_sel <= obj_seq_eval_gp_sel;
                obj_seq_pick2_code <= obj_seq_eval_code;
                obj_seq_pick2_next_code <= obj_seq_eval_next_code;
                obj_seq_pick2_next_valid <= obj_seq_eval_next_valid;
                obj_seq_pick2_mask <= obj_seq_eval_word_mask;
                obj_seq_pick2_bias <= obj_seq_eval_source_bias;
                obj_seq_pick2_row <= obj_seq_eval_row;
                obj_seq_pick2_color <= obj_seq_eval_attr[7:2];
                obj_seq_pick2_pri <= obj_seq_eval_attr[11:8];
                obj_seq_pick2_flipx <= obj_seq_eval_attr[12];
            end

            obj_seq_eval_valid <= obj_seq_scan_hit;
            obj_seq_eval_gp_sel <= obj_seq_scan_gp_sel;
            obj_seq_eval_attr <= obj_seq_scan_attr;
            obj_seq_eval_code_base <= obj_seq_scan_code_base;
            obj_seq_eval_dx <= obj_seq_scan_dx;
            obj_seq_eval_dy <= obj_seq_scan_dy;
            obj_seq_eval_w_tiles <= obj_seq_scan_w_tiles;
            obj_seq_eval_h_tiles <= obj_seq_scan_h_tiles;

            if (obj_seq_scan_gp1_exhausted || (obj_seq_scan_idx == 6'd63)) begin
                obj_seq_scan_active <= 1'b0;
                obj_seq_scan_done_pending <= 1'b1;
            end else if (obj_seq_scan_gp0_exhausted) begin
                obj_seq_scan_idx <= 6'd32;
            end else begin
                obj_seq_scan_idx <= obj_seq_scan_idx + 6'd1;
            end
        end else if (obj_seq_scan_done_pending) begin
            if (obj_seq_eval_wins_primary) begin
                if (obj_seq_pick_valid) begin
                    obj_seq_pick2_valid <= 1'b1;
                    obj_seq_pick2_gp_sel <= obj_seq_pick_gp_sel;
                    obj_seq_pick2_code <= obj_seq_pick_code;
                    obj_seq_pick2_next_code <= obj_seq_pick_next_code;
                    obj_seq_pick2_next_valid <= obj_seq_pick_next_valid;
                    obj_seq_pick2_mask <= obj_seq_pick_mask;
                    obj_seq_pick2_bias <= obj_seq_pick_bias;
                    obj_seq_pick2_row <= obj_seq_pick_row;
                    obj_seq_pick2_color <= obj_seq_pick_color;
                    obj_seq_pick2_pri <= obj_seq_pick_pri;
                    obj_seq_pick2_flipx <= obj_seq_pick_flipx;
                end
                obj_seq_pick_valid <= 1'b1;
                obj_seq_pick_gp_sel <= obj_seq_eval_gp_sel;
                obj_seq_pick_code <= obj_seq_eval_code;
                obj_seq_pick_next_code <= obj_seq_eval_next_code;
                obj_seq_pick_next_valid <= obj_seq_eval_next_valid;
                obj_seq_pick_mask <= obj_seq_eval_word_mask;
                obj_seq_pick_bias <= obj_seq_eval_source_bias;
                obj_seq_pick_row <= obj_seq_eval_row;
                obj_seq_pick_color <= obj_seq_eval_attr[7:2];
                obj_seq_pick_pri <= obj_seq_eval_attr[11:8];
                obj_seq_pick_flipx <= obj_seq_eval_attr[12];
            end else if (obj_seq_eval_wins_secondary) begin
                obj_seq_pick2_valid <= 1'b1;
                obj_seq_pick2_gp_sel <= obj_seq_eval_gp_sel;
                obj_seq_pick2_code <= obj_seq_eval_code;
                obj_seq_pick2_next_code <= obj_seq_eval_next_code;
                obj_seq_pick2_next_valid <= obj_seq_eval_next_valid;
                obj_seq_pick2_mask <= obj_seq_eval_word_mask;
                obj_seq_pick2_bias <= obj_seq_eval_source_bias;
                obj_seq_pick2_row <= obj_seq_eval_row;
                obj_seq_pick2_color <= obj_seq_eval_attr[7:2];
                obj_seq_pick2_pri <= obj_seq_eval_attr[11:8];
                obj_seq_pick2_flipx <= obj_seq_eval_attr[12];
            end

            obj_seq_eval_valid <= 1'b0;
            obj_seq_scan_done_pending <= 1'b0;
            obj_seq_pick_ready <= 1'b1;
            obj_seq_pick_target_x <= obj_seq_target_x;
            obj_seq_pick_target_y <= obj_seq_target_y;
        end

        if (obj_line_primary_commit_pending) begin
            if (obj_line_prefetch_space) begin
                if (!obj_line_prefetch_ready) begin
                    obj_line_prefetch_lo <= obj_line_primary_lo;
                    obj_line_prefetch_hi <= obj_line_primary_hi;
                    obj_line_prefetch_alt_lo <= 16'h0000;
                    obj_line_prefetch_alt_hi <= 16'h0000;
                    obj_line_prefetch_color <= obj_line_req_color;
                    obj_line_prefetch_pri <= obj_line_req_pri;
                    obj_line_prefetch_alt_color <= 7'h00;
                    obj_line_prefetch_alt_pri <= 4'h0;
                    obj_line_prefetch_x <= obj_line_req_target_x;
                    obj_line_prefetch_y <= obj_line_req_target_y;
                    obj_line_prefetch_valid <= obj_line_req_valid;
                    obj_line_prefetch_alt_valid <= 1'b0;
                    obj_line_prefetch_ready <= 1'b1;
                    obj_line_prefetch_gp_sel <= obj_line_req_gp_sel;
                    obj_line_prefetch_flipx <= obj_line_req_flipx;
                    obj_line_prefetch_alt_gp_sel <= 1'b0;
                    obj_line_prefetch_alt_flipx <= 1'b0;
                    obj_line_req_commit_slot <= 4'd0;
                end else if (!obj_line_prefetch1_ready) begin
                    obj_line_prefetch1_lo <= obj_line_primary_lo;
                    obj_line_prefetch1_hi <= obj_line_primary_hi;
                    obj_line_prefetch1_alt_lo <= 16'h0000;
                    obj_line_prefetch1_alt_hi <= 16'h0000;
                    obj_line_prefetch1_color <= obj_line_req_color;
                    obj_line_prefetch1_pri <= obj_line_req_pri;
                    obj_line_prefetch1_alt_color <= 7'h00;
                    obj_line_prefetch1_alt_pri <= 4'h0;
                    obj_line_prefetch1_x <= obj_line_req_target_x;
                    obj_line_prefetch1_y <= obj_line_req_target_y;
                    obj_line_prefetch1_valid <= obj_line_req_valid;
                    obj_line_prefetch1_alt_valid <= 1'b0;
                    obj_line_prefetch1_ready <= 1'b1;
                    obj_line_prefetch1_gp_sel <= obj_line_req_gp_sel;
                    obj_line_prefetch1_flipx <= obj_line_req_flipx;
                    obj_line_prefetch1_alt_gp_sel <= 1'b0;
                    obj_line_prefetch1_alt_flipx <= 1'b0;
                    obj_line_req_commit_slot <= 4'd1;
                end else if (!obj_line_prefetch2_ready) begin
                    obj_line_prefetch2_lo <= obj_line_primary_lo;
                    obj_line_prefetch2_hi <= obj_line_primary_hi;
                    obj_line_prefetch2_alt_lo <= 16'h0000;
                    obj_line_prefetch2_alt_hi <= 16'h0000;
                    obj_line_prefetch2_color <= obj_line_req_color;
                    obj_line_prefetch2_pri <= obj_line_req_pri;
                    obj_line_prefetch2_alt_color <= 7'h00;
                    obj_line_prefetch2_alt_pri <= 4'h0;
                    obj_line_prefetch2_x <= obj_line_req_target_x;
                    obj_line_prefetch2_y <= obj_line_req_target_y;
                    obj_line_prefetch2_valid <= obj_line_req_valid;
                    obj_line_prefetch2_alt_valid <= 1'b0;
                    obj_line_prefetch2_ready <= 1'b1;
                    obj_line_prefetch2_gp_sel <= obj_line_req_gp_sel;
                    obj_line_prefetch2_flipx <= obj_line_req_flipx;
                    obj_line_prefetch2_alt_gp_sel <= 1'b0;
                    obj_line_prefetch2_alt_flipx <= 1'b0;
                    obj_line_req_commit_slot <= 4'd2;
                end else if (obj_line_extra_free) begin
                    obj_line_prefetch_extra_lo[obj_line_extra_free_idx] <=
                        obj_line_primary_lo;
                    obj_line_prefetch_extra_hi[obj_line_extra_free_idx] <=
                        obj_line_primary_hi;
                    obj_line_prefetch_extra_alt_lo[obj_line_extra_free_idx] <= 16'h0000;
                    obj_line_prefetch_extra_alt_hi[obj_line_extra_free_idx] <= 16'h0000;
                    obj_line_prefetch_extra_color[obj_line_extra_free_idx] <=
                        obj_line_req_color;
                    obj_line_prefetch_extra_pri[obj_line_extra_free_idx] <=
                        obj_line_req_pri;
                    obj_line_prefetch_extra_alt_color[obj_line_extra_free_idx] <= 7'h00;
                    obj_line_prefetch_extra_alt_pri[obj_line_extra_free_idx] <= 4'h0;
                    obj_line_prefetch_extra_x[obj_line_extra_free_idx] <=
                        obj_line_req_target_x;
                    obj_line_prefetch_extra_y[obj_line_extra_free_idx] <=
                        obj_line_req_target_y;
                    obj_line_prefetch_extra_valid[obj_line_extra_free_idx] <=
                        obj_line_req_valid;
                    obj_line_prefetch_extra_alt_valid[obj_line_extra_free_idx] <= 1'b0;
                    obj_line_prefetch_extra_ready[obj_line_extra_free_idx] <= 1'b1;
                    obj_line_prefetch_extra_gp_sel[obj_line_extra_free_idx] <=
                        obj_line_req_gp_sel;
                    obj_line_prefetch_extra_flipx[obj_line_extra_free_idx] <=
                        obj_line_req_flipx;
                    obj_line_prefetch_extra_alt_gp_sel[obj_line_extra_free_idx] <= 1'b0;
                    obj_line_prefetch_extra_alt_flipx[obj_line_extra_free_idx] <= 1'b0;
                    obj_line_req_commit_slot <= 4'd3 + obj_line_extra_free_idx;
                end

                obj_line_req_primary_committed <= 1'b1;
                if (|(obj_line_primary_lo | obj_line_primary_hi)) begin
                    obj_line_seen_nonzero <= 1'b1;
                end
                if (obj_line_req_alt_valid) begin
                    obj_line_req_addr <= obj_line_req_alt_addr;
                    obj_line_req_phase <= 3'd4;
                end else begin
                    obj_line_req_pending <= 1'b0;
                    obj_line_req_phase <= 3'd0;
                    obj_line_req_primary_committed <= 1'b0;
                    obj_line_req_commit_slot <= OBJ_LINE_COMMIT_NONE;
                end
            end else begin
                obj_line_req_pending <= 1'b0;
                obj_line_req_phase <= 3'd0;
                obj_line_req_primary_committed <= 1'b0;
                obj_line_req_commit_slot <= OBJ_LINE_COMMIT_NONE;
            end
            obj_line_primary_commit_pending <= 1'b0;
        end else if (obj_line_alt_commit_pending) begin
            if (obj_line_req_primary_committed) begin
                case (obj_line_req_commit_slot)
                    4'd0: begin
                        if (obj_line_prefetch_ready &&
                            obj_line_prefetch_valid &&
                            (obj_line_prefetch_x == obj_line_req_target_x) &&
                            (obj_line_prefetch_y == obj_line_req_target_y)) begin
                            obj_line_prefetch_alt_lo <= obj_line_alt_lo;
                            obj_line_prefetch_alt_hi <= obj_line_alt_hi;
                            obj_line_prefetch_alt_color <= obj_line_req_alt_color;
                            obj_line_prefetch_alt_pri <= obj_line_req_alt_pri;
                            obj_line_prefetch_alt_valid <= obj_line_req_alt_valid;
                            obj_line_prefetch_alt_gp_sel <= obj_line_req_alt_gp_sel;
                            obj_line_prefetch_alt_flipx <= obj_line_req_alt_flipx;
                        end
                    end
                    4'd1: begin
                        if (obj_line_prefetch1_ready &&
                            obj_line_prefetch1_valid &&
                            (obj_line_prefetch1_x == obj_line_req_target_x) &&
                            (obj_line_prefetch1_y == obj_line_req_target_y)) begin
                            obj_line_prefetch1_alt_lo <= obj_line_alt_lo;
                            obj_line_prefetch1_alt_hi <= obj_line_alt_hi;
                            obj_line_prefetch1_alt_color <= obj_line_req_alt_color;
                            obj_line_prefetch1_alt_pri <= obj_line_req_alt_pri;
                            obj_line_prefetch1_alt_valid <= obj_line_req_alt_valid;
                            obj_line_prefetch1_alt_gp_sel <= obj_line_req_alt_gp_sel;
                            obj_line_prefetch1_alt_flipx <= obj_line_req_alt_flipx;
                        end
                    end
                    4'd2: begin
                        if (obj_line_prefetch2_ready &&
                            obj_line_prefetch2_valid &&
                            (obj_line_prefetch2_x == obj_line_req_target_x) &&
                            (obj_line_prefetch2_y == obj_line_req_target_y)) begin
                            obj_line_prefetch2_alt_lo <= obj_line_alt_lo;
                            obj_line_prefetch2_alt_hi <= obj_line_alt_hi;
                            obj_line_prefetch2_alt_color <= obj_line_req_alt_color;
                            obj_line_prefetch2_alt_pri <= obj_line_req_alt_pri;
                            obj_line_prefetch2_alt_valid <= obj_line_req_alt_valid;
                            obj_line_prefetch2_alt_gp_sel <= obj_line_req_alt_gp_sel;
                            obj_line_prefetch2_alt_flipx <= obj_line_req_alt_flipx;
                        end
                    end
                    default: begin
                        if ((obj_line_req_commit_slot >= 4'd3) &&
                            (obj_line_req_commit_slot <
                             (4'd3 + OBJ_LINE_EXTRA_PREFETCH_SLOTS)) &&
                            obj_line_prefetch_extra_ready[
                                obj_line_req_commit_slot - 4'd3] &&
                            obj_line_prefetch_extra_valid[
                                obj_line_req_commit_slot - 4'd3] &&
                            (obj_line_prefetch_extra_x[
                                obj_line_req_commit_slot - 4'd3] ==
                             obj_line_req_target_x) &&
                            (obj_line_prefetch_extra_y[
                                obj_line_req_commit_slot - 4'd3] ==
                             obj_line_req_target_y)) begin
                            obj_line_prefetch_extra_alt_lo[
                                obj_line_req_commit_slot - 4'd3] <= obj_line_alt_lo;
                            obj_line_prefetch_extra_alt_hi[
                                obj_line_req_commit_slot - 4'd3] <= obj_line_alt_hi;
                            obj_line_prefetch_extra_alt_color[
                                obj_line_req_commit_slot - 4'd3] <=
                                obj_line_req_alt_color;
                            obj_line_prefetch_extra_alt_pri[
                                obj_line_req_commit_slot - 4'd3] <=
                                obj_line_req_alt_pri;
                            obj_line_prefetch_extra_alt_valid[
                                obj_line_req_commit_slot - 4'd3] <=
                                obj_line_req_alt_valid;
                            obj_line_prefetch_extra_alt_gp_sel[
                                obj_line_req_commit_slot - 4'd3] <=
                                obj_line_req_alt_gp_sel;
                            obj_line_prefetch_extra_alt_flipx[
                                obj_line_req_commit_slot - 4'd3] <=
                                obj_line_req_alt_flipx;
                        end
                    end
                endcase
            end
            if (|(obj_line_alt_lo | obj_line_alt_hi)) begin
                obj_line_seen_nonzero <= 1'b1;
            end
            obj_line_req_pending <= 1'b0;
            obj_line_req_phase <= 3'd0;
            obj_line_req_primary_committed <= 1'b0;
            obj_line_req_commit_slot <= OBJ_LINE_COMMIT_NONE;
            obj_line_alt_commit_pending <= 1'b0;
        end else if (obj_line_req_pending &&
                     obj_line_slot_grant && gfx_obj_slot_ok) begin
            obj_line_seen_ok <= 1'b1;
            case (obj_line_req_phase)
                3'd0: begin
                    obj_line_fetch_lo <= gfx_obj_slot_dout;
                    obj_line_req_addr <= obj_line_req_high_addr;
                    obj_line_req_phase <= 3'd1;
                end
                3'd1: begin
                    if (obj_line_req_next_valid) begin
                        obj_line_tile0_lo <= obj_line_fetch_lo;
                        obj_line_tile0_hi <= gfx_obj_slot_dout;
                        obj_line_req_addr <= obj_line_req_next_addr;
                        obj_line_req_phase <= 3'd2;
                    end else begin
                        obj_line_primary_lo <= obj_line_composed_pixels[15:0];
                        obj_line_primary_hi <= obj_line_composed_pixels[31:16];
                        obj_line_primary_commit_pending <= 1'b1;
                    end
                end
                3'd2: begin
                    obj_line_fetch_lo <= gfx_obj_slot_dout;
                    obj_line_req_addr <= obj_line_req_next_high_addr;
                    obj_line_req_phase <= 3'd3;
                end
                3'd3: begin
                    obj_line_primary_lo <= obj_line_composed_pixels[15:0];
                    obj_line_primary_hi <= obj_line_composed_pixels[31:16];
                    obj_line_primary_commit_pending <= 1'b1;
                end
                3'd4: begin
                    obj_line_fetch_lo <= gfx_obj_slot_dout;
                    obj_line_req_addr <= obj_line_req_alt_high_addr;
                    obj_line_req_phase <= 3'd5;
                end
                3'd5: begin
                    if (obj_line_req_alt_next_valid) begin
                        obj_line_tile0_lo <= obj_line_fetch_lo;
                        obj_line_tile0_hi <= gfx_obj_slot_dout;
                        obj_line_req_addr <= obj_line_req_alt_next_addr;
                        obj_line_req_phase <= 3'd6;
                    end else begin
                        obj_line_alt_lo <= obj_line_composed_pixels[15:0];
                        obj_line_alt_hi <= obj_line_composed_pixels[31:16];
                        obj_line_alt_commit_pending <= 1'b1;
                    end
                end
                3'd6: begin
                    obj_line_fetch_lo <= gfx_obj_slot_dout;
                    obj_line_req_addr <= obj_line_req_alt_next_high_addr;
                    obj_line_req_phase <= 3'd7;
                end
                default: begin
                    obj_line_alt_lo <= obj_line_composed_pixels[15:0];
                    obj_line_alt_hi <= obj_line_composed_pixels[31:16];
                    obj_line_alt_commit_pending <= 1'b1;
                end
            endcase
        end
    end
end

always @(posedge clk) begin
    if (video_runtime_reset || gp_tile_hex_mode ||
        !debug_object_cache_en) begin
        obj_gfx_fetch_slot <= 5'd0;
        obj_gfx_fetch_tile <= obj_gfx_debug_tile_page;
        obj_gfx_fetch_row <= 3'd0;
        obj_gfx_fetch_phase <= 2'd0;
        obj_gfx_fetch_done <= 1'b0;
        obj_gfx_req_pending <= 1'b0;
        obj_gfx_req_addr <= {AW{1'b0}};
        obj_gfx_fetch_lo <= 16'h0000;
        obj_gfx_valid_count <= 13'd0;
        obj_gfx_nonzero_count <= 13'd0;
        obj_gfx_seen_ok <= 1'b0;
        obj_gfx_seen_nonzero <= 1'b0;
        obj_gfx_debug_slot_valid <= 1'b0;
        obj_gfx_debug_slot <= 5'd0;
        obj_gfx_debug_tile <= 8'd0;
        obj_gfx_debug_hit_valid <= 1'b0;
        obj_gfx_debug_hit_row <= 3'd0;
        obj_gfx_debug_hit_lo <= 16'h0000;
        obj_gfx_debug_hit_hi <= 16'h0000;
        obj_gfx_debug_valid <= 8'h00;
    end else if (debug_frame_tick) begin
        obj_gfx_fetch_slot <= 5'd0;
        obj_gfx_fetch_tile <= obj_gfx_debug_tile_page;
        obj_gfx_fetch_row <= 3'd0;
        obj_gfx_fetch_phase <= 2'd0;
        obj_gfx_fetch_done <= 1'b0;
        obj_gfx_req_pending <= 1'b0;
        obj_gfx_req_addr <= {AW{1'b0}};
        obj_gfx_fetch_lo <= 16'h0000;
        obj_gfx_debug_hit_valid <= 1'b0;
        obj_gfx_debug_slot_valid <= 1'b0;
        obj_gfx_debug_slot <= 5'd0;
        obj_gfx_debug_tile <= 8'd0;
        obj_gfx_debug_valid <= 8'h00;
    end else if (!obj_gfx_fetch_done && (obj_scan_phase == 4'd10)) begin
        case (obj_gfx_fetch_phase)
            2'd0: begin
                obj_gfx_req_pending <= 1'b0;
                if (obj_gfx_fetch_tile_live) begin
                    obj_gfx_req_addr <= obj_gfx_fetch_addr_lo;
                    obj_gfx_req_pending <= 1'b1;
                    obj_gfx_fetch_phase <= 2'd1;
                end else if (obj_gfx_fetch_slot == 5'd31) begin
                    obj_gfx_fetch_done <= 1'b1;
                end else begin
                    obj_gfx_fetch_row <= 3'd0;
                    obj_gfx_fetch_tile <= obj_gfx_debug_tile_page;
                    obj_gfx_fetch_slot <= obj_gfx_fetch_slot + 5'd1;
                end
            end

            2'd1: begin
                if (obj_gfx_slot_grant && gfx_obj_slot_ok) begin
                    obj_gfx_seen_ok <= 1'b1;
                    obj_gfx_fetch_lo <= gfx_obj_slot_dout;
                    obj_gfx_req_addr <= obj_gfx_fetch_addr_hi;
                    obj_gfx_fetch_phase <= 2'd2;
                end
            end

            default: begin
                if (obj_gfx_slot_grant && gfx_obj_slot_ok) begin
                    obj_gfx_seen_ok <= 1'b1;
                    if (obj_gfx_valid_count != 13'h1fff) begin
                        obj_gfx_valid_count <= obj_gfx_valid_count + 13'd1;
                    end
                    if (|(obj_gfx_fetch_lo | gfx_obj_slot_dout)) begin
                        obj_gfx_seen_nonzero <= 1'b1;
                        if (!obj_gfx_debug_hit_valid) begin
                            obj_gfx_debug_hit_valid <= 1'b1;
                            obj_gfx_debug_hit_row <= obj_gfx_fetch_row;
                            obj_gfx_debug_hit_lo <= obj_gfx_fetch_lo;
                            obj_gfx_debug_hit_hi <= gfx_obj_slot_dout;
                        end
                        if (obj_gfx_nonzero_count != 13'h1fff) begin
                            obj_gfx_nonzero_count <= obj_gfx_nonzero_count + 13'd1;
                        end
                        if (!obj_gfx_debug_slot_valid) begin
                            obj_gfx_debug_slot_valid <= 1'b1;
                            obj_gfx_debug_slot <= obj_gfx_fetch_slot;
                            obj_gfx_debug_tile <= obj_gfx_fetch_tile;
                        end
                    end
                    if (((!obj_gfx_debug_slot_valid) && |(obj_gfx_fetch_lo | gfx_obj_slot_dout)) ||
                        (obj_gfx_debug_slot_valid &&
                         (obj_gfx_fetch_slot == obj_gfx_debug_slot) &&
                         (obj_gfx_fetch_tile == obj_gfx_debug_tile))) begin
                        obj_gfx_debug_lo[obj_gfx_fetch_row] <= obj_gfx_fetch_lo;
                        obj_gfx_debug_hi[obj_gfx_fetch_row] <= gfx_obj_slot_dout;
                        obj_gfx_debug_valid[obj_gfx_fetch_row] <= 1'b1;
                    end
                    obj_gfx_req_pending <= 1'b0;
                    obj_gfx_fetch_phase <= 2'd0;

                    if ((obj_gfx_fetch_slot == 5'd31) &&
                        (obj_gfx_fetch_tile[3:0] == 4'd15) &&
                        (obj_gfx_fetch_row == 3'd7)) begin
                        obj_gfx_fetch_done <= 1'b1;
                    end else if (obj_gfx_fetch_row == 3'd7) begin
                        obj_gfx_fetch_row <= 3'd0;
                        if (obj_gfx_fetch_tile[3:0] == 4'd15) begin
                            obj_gfx_fetch_tile <= obj_gfx_debug_tile_page;
                            obj_gfx_fetch_slot <= obj_gfx_fetch_slot + 5'd1;
                        end else begin
                            obj_gfx_fetch_tile <= obj_gfx_fetch_tile + 8'd1;
                        end
                    end else begin
                        obj_gfx_fetch_row <= obj_gfx_fetch_row + 3'd1;
                    end
                end
            end
        endcase
    end
end

always @(posedge clk) begin
    if (cpu_reset) begin
        debug_live0_px <= 16'h0000;
        debug_live1_px <= 16'h0000;
        debug_live2_px <= 16'h0000;
        debug_live3_px <= 16'h0000;
        debug_live4_px <= 16'h0000;
        debug_live5_px <= 16'h0000;
        debug_live6_px <= 16'h0000;
        debug_live7_px <= 16'h0000;
    end else if (clkdiv == 4'd13) begin
        debug_live0_px <= debug_live0;
        debug_live1_px <= debug_live1;
        debug_live2_px <= debug_live2;
        debug_live3_px <= debug_live3;
        debug_live4_px <= debug_live4;
        debug_live5_px <= debug_live5;
        debug_live6_px <= debug_live6;
        debug_live7_px <= debug_live7;
    end
end

always @(posedge clk) begin
    if (cpu_reset) begin
        debug_hold0 <= 16'h0000;
        debug_hold1 <= 16'h0000;
        debug_hold2 <= 16'h0000;
        debug_hold3 <= 16'h0000;
        debug_hold4 <= 16'h0000;
        debug_hold5 <= 16'h0000;
        debug_hold6 <= 16'h0000;
        debug_hold7 <= 16'h0000;
        debug_hold_div <= 6'd0;
        gp0_sample_addr <= 13'h0000;
        gp1_sample_addr <= 13'h0000;
        gp0_sample_word <= 16'h0000;
        gp1_sample_word <= 16'h0000;
    end else if (debug_frame_tick) begin
        gp0_sample_addr <= gp0_dbg_ptr - 13'd1;
        gp1_sample_addr <= gp1_dbg_ptr - 13'd1;
        gp0_sample_word <= gp0_scan_dout;
        gp1_sample_word <= gp1_scan_dout;

        if (debug_hold_div == 6'd0) begin
            debug_hold0 <= debug_live0_px;
            debug_hold1 <= debug_live1_px;
            debug_hold2 <= debug_live2_px;
            debug_hold3 <= debug_live3_px;
            debug_hold4 <= debug_live4_px;
            debug_hold5 <= debug_live5_px;
            debug_hold6 <= debug_live6_px;
            debug_hold7 <= debug_live7_px;
            debug_hold_div <= 6'd59;
        end else begin
            debug_hold_div <= debug_hold_div - 6'd1;
        end
    end
end

always @(*) begin
    case (rom_probe_row)
        3'd0: cpu_display_word = debug_hold0;
        3'd1: cpu_display_word = debug_hold1;
        3'd2: cpu_display_word = debug_hold2;
        3'd3: cpu_display_word = debug_hold3;
        3'd4: cpu_display_word = debug_hold4;
        3'd5: cpu_display_word = debug_hold5;
        3'd6: cpu_display_word = debug_hold6;
        default: cpu_display_word = debug_hold7;
    endcase

    if (rom_probe_match) begin
        rom_probe_display_word = cpu_display_word;
    end else begin
        case (rom_probe_row)
            3'd0: rom_probe_display_word = diag_flags;
            3'd1: rom_probe_display_word = {load_seen, rom_probe_seen};
            3'd2: rom_probe_display_word = {diag_ioctl_wr_count, diag_last_ioctl_dout};
            3'd3: rom_probe_display_word = diag_last_ioctl_addr;
            3'd4: rom_probe_display_word = load_word0;
            3'd5: rom_probe_display_word = load_word3;
            3'd6: rom_probe_display_word = rom_probe_word0;
            default: rom_probe_display_word = rom_probe_word3;
        endcase
    end
end

always @(posedge clk) begin
    last_ioctl_rom <= ioctl_rom;

    if (ioctl_rom && !last_ioctl_rom) begin
        load_word0 <= 16'h0000;
        load_word1 <= 16'h0000;
        load_word2 <= 16'h0000;
        load_word3 <= 16'h0000;
        load_seen <= 8'h00;
        ever_ioctl_rom <= 1'b0;
        ever_ioctl_wr <= 1'b0;
        ever_dwnld_busy <= 1'b0;
        ever_prog_we <= 1'b0;
        ever_prog_ack <= 1'b0;
        ever_rom_probe_rd <= 1'b0;
        ever_rom_probe_ok <= 1'b0;
        diag_ioctl_wr_count <= 8'h00;
        diag_last_ioctl_dout <= 8'h00;
        diag_last_ioctl_addr <= 16'h0000;
    end

    if (ioctl_rom) ever_ioctl_rom <= 1'b1;
    if (ioctl_wr) begin
        ever_ioctl_wr <= 1'b1;
        diag_ioctl_wr_count <= diag_ioctl_wr_count + 8'd1;
        diag_last_ioctl_dout <= ioctl_dout;
        diag_last_ioctl_addr <= ioctl_addr[15:0];
    end
    if (dwnld_busy) ever_dwnld_busy <= 1'b1;
    if (prog_we) ever_prog_we <= 1'b1;
    if (prog_ack) ever_prog_ack <= 1'b1;
    if (rom_slot_rd) ever_rom_probe_rd <= 1'b1;
    if (rom_slot_ok) ever_rom_probe_ok <= 1'b1;

    if (ioctl_rom && ioctl_wr && (ioctl_addr[26:3] == 24'd0)) begin
        case (ioctl_addr[2:0])
            3'd0: load_word0[7:0]  <= ioctl_dout;
            3'd1: load_word0[15:8] <= ioctl_dout;
            3'd2: load_word1[7:0]  <= ioctl_dout;
            3'd3: load_word1[15:8] <= ioctl_dout;
            3'd4: load_word2[7:0]  <= ioctl_dout;
            3'd5: load_word2[15:8] <= ioctl_dout;
            3'd6: load_word3[7:0]  <= ioctl_dout;
            default: load_word3[15:8] <= ioctl_dout;
        endcase
        load_seen[ioctl_addr[2:0]] <= 1'b1;
    end
end

always @(posedge clk) begin
    if (rst || dwnld_busy || ioctl_rom) begin
        rom_probe_addr <= 2'd0;
        rom_probe_cs <= 1'b0;
        rom_probe_idx <= 2'd0;
        rom_probe_seen <= 8'h00;
        rom_probe_word0 <= 16'h0000;
        rom_probe_word1 <= 16'h0000;
        rom_probe_word2 <= 16'h0000;
        rom_probe_word3 <= 16'h0000;
    end else if (!rom_probe_done) begin
        if (!rom_probe_cs) begin
            rom_probe_addr <= rom_probe_idx;
            rom_probe_cs <= 1'b1;
        end else if (rom_slot_ok) begin
            case (rom_probe_idx)
                2'd0: begin
                    rom_probe_word0 <= rom_slot_dout;
                    rom_probe_seen[1:0] <= 2'b11;
                end
                2'd1: begin
                    rom_probe_word1 <= rom_slot_dout;
                    rom_probe_seen[3:2] <= 2'b11;
                end
                2'd2: begin
                    rom_probe_word2 <= rom_slot_dout;
                    rom_probe_seen[5:4] <= 2'b11;
                end
                default: begin
                    rom_probe_word3 <= rom_slot_dout;
                    rom_probe_seen[7:6] <= 2'b11;
                end
            endcase
            rom_probe_idx <= rom_probe_idx + 2'd1;
            rom_probe_cs <= 1'b0;
        end
    end
end

always @(posedge clk) begin
    if (rst || dwnld_busy || ioctl_rom) begin
        gfx_fixed_probe_addr <= {AW{1'b0}};
        gfx_fixed_probe_cs <= 1'b0;
        gfx_fixed_probe_idx <= 3'd0;
        gfx_fixed_probe_seen <= 6'b000000;
        gfx_fixed_probe_word0 <= 16'h0000;
        gfx_fixed_probe_word1 <= 16'h0000;
        gfx_fixed_probe_word2 <= 16'h0000;
        gfx_fixed_probe_word3 <= 16'h0000;
        gfx_fixed_probe_word4 <= 16'h0000;
        gfx_fixed_probe_word5 <= 16'h0000;
    end else if (!gp_rom_probe_mode) begin
        gfx_fixed_probe_cs <= 1'b0;
        gfx_fixed_probe_idx <= 3'd0;
        gfx_fixed_probe_seen <= 6'b000000;
    end else if (!gfx_fixed_probe_cs) begin
        gfx_fixed_probe_addr <= gfx_fixed_probe_next_addr;
        gfx_fixed_probe_cs <= 1'b1;
    end else if (gfx_probe_slot_ok) begin
        case (gfx_fixed_probe_idx)
            3'd0: begin
                gfx_fixed_probe_word0 <= gfx_probe_slot_word;
                gfx_fixed_probe_seen[0] <= 1'b1;
            end
            3'd1: begin
                gfx_fixed_probe_word1 <= gfx_probe_slot_word;
                gfx_fixed_probe_seen[1] <= 1'b1;
            end
            3'd2: begin
                gfx_fixed_probe_word2 <= gfx_probe_slot_word;
                gfx_fixed_probe_seen[2] <= 1'b1;
            end
            3'd3: begin
                gfx_fixed_probe_word3 <= gfx_probe_slot_word;
                gfx_fixed_probe_seen[3] <= 1'b1;
            end
            3'd4: begin
                gfx_fixed_probe_word4 <= gfx_probe_slot_word;
                gfx_fixed_probe_seen[4] <= 1'b1;
            end
            default: begin
                gfx_fixed_probe_word5 <= gfx_probe_slot_word;
                gfx_fixed_probe_seen[5] <= 1'b1;
            end
        endcase
        gfx_fixed_probe_cs <= 1'b0;
        gfx_fixed_probe_idx <= (gfx_fixed_probe_idx == 3'd5) ? 3'd0 : (gfx_fixed_probe_idx + 3'd1);
    end
end

task comp_latch_prefetch;
    input [2:0] slot;
    input [4:0] cache_idx;
    begin
        if (comp_prefetch_ready[slot] && comp_word_cache_valid[cache_idx]) begin
            comp_latched_lo[slot] <= comp_word_cache_lo[cache_idx];
            comp_latched_hi[slot] <= comp_word_cache_hi[cache_idx];
            comp_latched_color[slot] <= comp_word_cache_color[cache_idx];
            comp_latched_pri[slot] <= comp_word_cache_pri[cache_idx];
            comp_latched_valid[slot] <= 1'b1;
        end else if (comp_prefetch_ready[slot]) begin
            comp_latched_valid[slot] <= 1'b0;
        end else if (!COMP_MISS_HOLD_LAST_TEST) begin
            comp_latched_valid[slot] <= 1'b0;
        end

        if (comp_fetch_pending[slot]) begin
            if (comp_fetch_deep_stage[slot]) begin
                comp_fetch_deep_stage[slot] <= 1'b0;
                comp_fetch_far_stage[slot] <= 1'b1;
            end else if (comp_fetch_far_stage[slot]) begin
                comp_fetch_far_stage[slot] <= 1'b0;
                comp_fetch_stage[slot] <= 1'b1;
            end else if (comp_fetch_stage[slot]) begin
                comp_fetch_stage[slot] <= 1'b0;
            end else begin
                comp_fetch_pending[slot] <= 1'b0;
            end
        end

        if (gfx_req_pending && (gfx_req_slot == slot)) begin
            if (gfx_req_deep_stage) begin
                gfx_req_deep_stage <= 1'b0;
                gfx_req_far_stage <= 1'b1;
            end else if (gfx_req_far_stage) begin
                gfx_req_far_stage <= 1'b0;
                gfx_req_stage <= 1'b1;
            end else if (gfx_req_stage) begin
                gfx_req_stage <= 1'b0;
            end else begin
                gfx_req_pending <= 1'b0;
                gfx_req_phase <= 1'b0;
            end
        end
        if (gfx_req_probe_pending && (gfx_req_probe_slot == slot)) begin
            if (gfx_req_probe_deep_stage) begin
                gfx_req_probe_deep_stage <= 1'b0;
                gfx_req_probe_far_stage <= 1'b1;
            end else if (gfx_req_probe_far_stage) begin
                gfx_req_probe_far_stage <= 1'b0;
                gfx_req_probe_stage <= 1'b1;
            end else if (gfx_req_probe_stage) begin
                gfx_req_probe_stage <= 1'b0;
            end else begin
                gfx_req_probe_pending <= 1'b0;
                gfx_req_probe_phase <= 1'b0;
            end
        end
    end
endtask

task comp_store_prefetch;
    input [2:0] slot;
    reg [4:0] target_idx;
    begin
        target_idx = {slot, gfx_req_target_x[4:3]};
        if ((comp_word_cache_target_x[target_idx] == gfx_req_target_x) &&
            (comp_word_cache_target_y[target_idx] == gfx_req_target_y)) begin
            comp_word_cache_lo[target_idx] <= gfx_fetch_lo;
            comp_word_cache_hi[target_idx] <= gfx_scroll_slot_dout;
            comp_word_cache_color[target_idx] <= gfx_req_color;
            comp_word_cache_pri[target_idx] <= gfx_req_pri;
            comp_word_cache_valid[target_idx] <= gfx_req_valid;
            comp_word_cache_ready[target_idx] <= 1'b1;
        end
    end
endtask

task comp_store_prefetch_probe;
    input [2:0] slot;
    reg [4:0] target_idx;
    begin
        target_idx = {slot, gfx_req_probe_target_x[4:3]};
        if ((comp_word_cache_target_x[target_idx] == gfx_req_probe_target_x) &&
            (comp_word_cache_target_y[target_idx] == gfx_req_probe_target_y)) begin
            comp_word_cache_lo[target_idx] <= gfx_fetch_probe_lo;
            comp_word_cache_hi[target_idx] <= gfx_probe_slot_dout;
            comp_word_cache_color[target_idx] <= gfx_req_probe_color;
            comp_word_cache_pri[target_idx] <= gfx_req_probe_pri;
            comp_word_cache_valid[target_idx] <= gfx_req_probe_valid;
            comp_word_cache_ready[target_idx] <= 1'b1;
        end
    end
endtask

always @(posedge clk) begin
    if (video_runtime_reset || gp_tile_hex_mode) begin
        gfx_req_addr <= {AW{1'b0}};
        gfx_req_high_addr <= {AW{1'b0}};
        gfx_req_target_x <= 9'd0;
        gfx_req_target_y <= 9'd0;
        gfx_req_pending <= 1'b0;
        gfx_req_phase <= 1'b0;
        gfx_req_stage <= 1'b0;
        gfx_req_far_stage <= 1'b0;
        gfx_req_deep_stage <= 1'b0;
        gfx_req_valid <= 1'b0;
        gfx_req_color <= 7'h00;
        gfx_req_pri <= 4'h0;
        gfx_req_slot <= 3'd0;
        gfx_fetch_lo <= 16'h0000;
        gfx_req_probe_addr <= {AW{1'b0}};
        gfx_req_probe_high_addr <= {AW{1'b0}};
        gfx_req_probe_target_x <= 9'd0;
        gfx_req_probe_target_y <= 9'd0;
        gfx_req_probe_pending <= 1'b0;
        gfx_req_probe_phase <= 1'b0;
        gfx_req_probe_stage <= 1'b0;
        gfx_req_probe_far_stage <= 1'b0;
        gfx_req_probe_deep_stage <= 1'b0;
        gfx_req_probe_valid <= 1'b0;
        gfx_req_probe_color <= 7'h00;
        gfx_req_probe_pri <= 4'h0;
        gfx_req_probe_slot <= 3'd3;
        gfx_fetch_probe_lo <= 16'h0000;
        gfx_seen_ok <= 1'b0;
        gp0_comp_scan_addr <= 13'h0000;
        gp1_comp_scan_addr <= 13'h0000;
        comp_fetch_pending <= 6'b000000;
        comp_fetch_stage <= 6'b000000;
        comp_fetch_far_stage <= 6'b000000;
        comp_fetch_deep_stage <= 6'b000000;
        comp_word_start_event <= 6'b000000;
        comp_fetch_rr <= 3'd0;
        comp_fetch_probe_rr <= 3'd3;
        comp_fetch_grant_slot <= 3'd0;
        comp_fetch_grant_valid <= 1'b0;
        comp_fetch_probe_grant_slot <= 3'd3;
        comp_fetch_probe_grant_valid <= 1'b0;
        comp_frame_bank <= 1'b0;
        comp_latched_valid <= 6'b000000;
        for (comp_i = 0; comp_i < 6; comp_i = comp_i + 1) begin
            comp_tile_attr[comp_i] <= 16'h0000;
            comp_tile_code[comp_i] <= 16'h0000;
            comp_fetch_addr_lo[comp_i] <= {AW{1'b0}};
            comp_fetch_addr_hi[comp_i] <= {AW{1'b0}};
            comp_fetch_target_x[comp_i] <= 9'd0;
            comp_fetch_target_y[comp_i] <= 9'd0;
            comp_fetch_color[comp_i] <= 7'h00;
            comp_fetch_pri[comp_i] <= 4'h0;
            comp_latched_lo[comp_i] <= 16'h0000;
            comp_latched_hi[comp_i] <= 16'h0000;
            comp_latched_color[comp_i] <= 7'h00;
            comp_latched_pri[comp_i] <= 4'h0;
        end
        for (comp_cache_i = 0; comp_cache_i < 24; comp_cache_i = comp_cache_i + 1) begin
            comp_word_cache_lo[comp_cache_i] <= 16'h0000;
            comp_word_cache_hi[comp_cache_i] <= 16'h0000;
            comp_word_cache_color[comp_cache_i] <= 7'h00;
            comp_word_cache_pri[comp_cache_i] <= 4'h0;
            comp_word_cache_target_x[comp_cache_i] <= 9'd0;
            comp_word_cache_target_y[comp_cache_i] <= 9'd0;
            comp_word_cache_valid[comp_cache_i] <= 1'b0;
            comp_word_cache_ready[comp_cache_i] <= 1'b0;
        end
    end else begin
        if (COMP_BANKED_SCROLL_DIAG && debug_frame_tick) begin
            comp_frame_bank <= ~comp_frame_bank;
            gfx_req_pending <= 1'b0;
            gfx_req_phase <= 1'b0;
            gfx_req_stage <= 1'b0;
            gfx_req_far_stage <= 1'b0;
            gfx_req_deep_stage <= 1'b0;
            gfx_req_probe_pending <= 1'b0;
            gfx_req_probe_phase <= 1'b0;
            gfx_req_probe_stage <= 1'b0;
            gfx_req_probe_far_stage <= 1'b0;
            gfx_req_probe_deep_stage <= 1'b0;
            comp_fetch_pending <= 6'b000000;
            comp_fetch_stage <= 6'b000000;
            comp_fetch_far_stage <= 6'b000000;
            comp_fetch_deep_stage <= 6'b000000;
            comp_word_start_event <= 6'b000000;
            comp_fetch_grant_valid <= 1'b0;
            comp_fetch_probe_grant_valid <= 1'b0;
            comp_latched_valid <= 6'b000000;
            for (comp_cache_i = 0; comp_cache_i < 24; comp_cache_i = comp_cache_i + 1) begin
                comp_word_cache_valid[comp_cache_i] <= 1'b0;
                comp_word_cache_ready[comp_cache_i] <= 1'b0;
            end
        end

        if (clkdiv == 4'd0) begin
            comp_word_start_event <= comp_word_start_mask;
        end else if (clkdiv == 4'd1) begin
            comp_word_start_event <= 6'b000000;
            if (comp_word_start_event[0]) comp_latch_prefetch(3'd0, comp_word_cache_idx0);
            if (comp_word_start_event[1]) comp_latch_prefetch(3'd1, comp_word_cache_idx1);
            if (comp_word_start_event[2]) comp_latch_prefetch(3'd2, comp_word_cache_idx2);
            if (comp_word_start_event[3]) comp_latch_prefetch(3'd3, comp_word_cache_idx3);
            if (comp_word_start_event[4]) comp_latch_prefetch(3'd4, comp_word_cache_idx4);
            if (comp_word_start_event[5]) comp_latch_prefetch(3'd5, comp_word_cache_idx5);
        end

        if (comp_fetch_grant_valid) begin
            comp_fetch_grant_valid <= 1'b0;
        end else if (comp_fetch_primary_can_grant) begin
            comp_fetch_grant_slot <= comp_fetch_pick_slot;
            comp_fetch_grant_valid <= 1'b1;
        end

        if (comp_fetch_probe_grant_valid) begin
            comp_fetch_probe_grant_valid <= 1'b0;
        end else if (comp_fetch_probe_can_grant) begin
            comp_fetch_probe_grant_slot <= comp_fetch_probe_pick_slot;
            comp_fetch_probe_grant_valid <= 1'b1;
        end

        case (clkdiv)
            4'd0: begin
                gp0_comp_scan_addr <= gp0_l0_pair_addr;
                gp1_comp_scan_addr <= gp1_l0_pair_addr;
            end
            4'd2: begin
                comp_tile_attr[0] <= gp0_scan_dout;
                comp_tile_attr[3] <= gp1_scan_dout;
                gp0_comp_scan_addr <= gp0_l0_pair_addr + 13'd1;
                gp1_comp_scan_addr <= gp1_l0_pair_addr + 13'd1;
            end
            4'd4: begin
                comp_tile_code[0] <= gp0_scan_dout;
                comp_tile_code[3] <= gp1_scan_dout;
                gp0_comp_scan_addr <= gp0_l1_pair_addr;
                gp1_comp_scan_addr <= gp1_l1_pair_addr;
            end
            4'd6: begin
                comp_tile_attr[1] <= gp0_scan_dout;
                comp_tile_attr[4] <= gp1_scan_dout;
                gp0_comp_scan_addr <= gp0_l1_pair_addr + 13'd1;
                gp1_comp_scan_addr <= gp1_l1_pair_addr + 13'd1;
            end
            4'd8: begin
                comp_tile_code[1] <= gp0_scan_dout;
                comp_tile_code[4] <= gp1_scan_dout;
                gp0_comp_scan_addr <= gp0_l2_pair_addr;
                gp1_comp_scan_addr <= gp1_l2_pair_addr;
            end
            4'd10: begin
                comp_tile_attr[2] <= gp0_scan_dout;
                comp_tile_attr[5] <= gp1_scan_dout;
                gp0_comp_scan_addr <= gp0_l2_pair_addr + 13'd1;
                gp1_comp_scan_addr <= gp1_l2_pair_addr + 13'd1;
            end
            4'd12: begin
                comp_tile_code[2] <= gp0_scan_dout;
                comp_tile_code[5] <= gp1_scan_dout;
            end
            default: begin
            end
        endcase

        if (gfx_req_deadline_miss) begin
            gfx_req_pending <= 1'b0;
            gfx_req_phase <= 1'b0;
            gfx_req_stage <= 1'b0;
            gfx_req_far_stage <= 1'b0;
            gfx_req_deep_stage <= 1'b0;
        end else if (gfx_req_pending && gfx_scroll_slot_ok) begin
            if (!gfx_req_phase) begin
                gfx_fetch_lo <= gfx_scroll_slot_dout;
                gfx_req_phase <= 1'b1;
            end else begin
                comp_store_prefetch(gfx_req_slot);
                gfx_seen_ok <= 1'b1;
                gfx_req_pending <= 1'b0;
                gfx_req_phase <= 1'b0;
                gfx_req_stage <= 1'b0;
                gfx_req_far_stage <= 1'b0;
                gfx_req_deep_stage <= 1'b0;
            end
        end else if (comp_fetch_grant_launch) begin
            gfx_req_addr <= comp_fetch_addr_lo[comp_fetch_grant_slot];
            gfx_req_high_addr <= comp_fetch_addr_hi[comp_fetch_grant_slot];
            gfx_req_target_x <= comp_fetch_target_x[comp_fetch_grant_slot];
            gfx_req_target_y <= comp_fetch_target_y[comp_fetch_grant_slot];
            gfx_req_color <= comp_fetch_color[comp_fetch_grant_slot];
            gfx_req_pri <= comp_fetch_pri[comp_fetch_grant_slot];
            gfx_req_valid <= 1'b1;
            gfx_req_slot <= comp_fetch_grant_slot;
            gfx_req_stage <= comp_fetch_stage[comp_fetch_grant_slot];
            gfx_req_far_stage <= comp_fetch_far_stage[comp_fetch_grant_slot];
            gfx_req_deep_stage <= comp_fetch_deep_stage[comp_fetch_grant_slot];
            gfx_req_pending <= 1'b1;
            gfx_req_phase <= 1'b0;
            comp_fetch_pending[comp_fetch_grant_slot] <= 1'b0;
            comp_fetch_stage[comp_fetch_grant_slot] <= 1'b0;
            comp_fetch_far_stage[comp_fetch_grant_slot] <= 1'b0;
            comp_fetch_deep_stage[comp_fetch_grant_slot] <= 1'b0;
            comp_fetch_rr <= (comp_fetch_grant_slot == 3'd2) ?
                             3'd0 : (comp_fetch_grant_slot + 3'd1);
        end

        if (gp_rom_probe_mode || gfx_req_probe_deadline_miss) begin
            gfx_req_probe_pending <= 1'b0;
            gfx_req_probe_phase <= 1'b0;
            gfx_req_probe_stage <= 1'b0;
            gfx_req_probe_far_stage <= 1'b0;
            gfx_req_probe_deep_stage <= 1'b0;
        end else if (gfx_req_probe_pending && gfx_probe_slot_ok) begin
            if (!gfx_req_probe_phase) begin
                gfx_fetch_probe_lo <= gfx_probe_slot_dout;
                gfx_req_probe_phase <= 1'b1;
            end else begin
                comp_store_prefetch_probe(gfx_req_probe_slot);
                gfx_seen_ok <= 1'b1;
                gfx_req_probe_pending <= 1'b0;
                gfx_req_probe_phase <= 1'b0;
                gfx_req_probe_stage <= 1'b0;
                gfx_req_probe_far_stage <= 1'b0;
                gfx_req_probe_deep_stage <= 1'b0;
            end
        end else if (comp_fetch_probe_grant_launch) begin
            gfx_req_probe_addr <= comp_fetch_addr_lo[comp_fetch_probe_grant_slot];
            gfx_req_probe_high_addr <= comp_fetch_addr_hi[comp_fetch_probe_grant_slot];
            gfx_req_probe_target_x <= comp_fetch_target_x[comp_fetch_probe_grant_slot];
            gfx_req_probe_target_y <= comp_fetch_target_y[comp_fetch_probe_grant_slot];
            gfx_req_probe_color <= comp_fetch_color[comp_fetch_probe_grant_slot];
            gfx_req_probe_pri <= comp_fetch_pri[comp_fetch_probe_grant_slot];
            gfx_req_probe_valid <= 1'b1;
            gfx_req_probe_slot <= comp_fetch_probe_grant_slot;
            gfx_req_probe_stage <= comp_fetch_stage[comp_fetch_probe_grant_slot];
            gfx_req_probe_far_stage <= comp_fetch_far_stage[comp_fetch_probe_grant_slot];
            gfx_req_probe_deep_stage <= comp_fetch_deep_stage[comp_fetch_probe_grant_slot];
            gfx_req_probe_pending <= 1'b1;
            gfx_req_probe_phase <= 1'b0;
            comp_fetch_pending[comp_fetch_probe_grant_slot] <= 1'b0;
            comp_fetch_stage[comp_fetch_probe_grant_slot] <= 1'b0;
            comp_fetch_far_stage[comp_fetch_probe_grant_slot] <= 1'b0;
            comp_fetch_deep_stage[comp_fetch_probe_grant_slot] <= 1'b0;
            comp_fetch_probe_rr <= (comp_fetch_probe_grant_slot >= 3'd5) ?
                                   3'd3 : (comp_fetch_probe_grant_slot + 3'd1);
        end

        if (clkdiv == 4'd5) begin
            if (gp0_l0_word_start) begin
                comp_fetch_addr_lo[0] <= comp_addr_lo0;
                comp_fetch_addr_hi[0] <= comp_addr_hi0;
                comp_fetch_target_x[0] <= gp_comp_fetch_target_x;
                comp_fetch_target_y[0] <= gp_comp_vcnt;
                comp_fetch_color[0] <= comp_attr_eff0[6:0];
                comp_fetch_pri[0] <= comp_pri0;
                comp_fetch_pending[0] <= comp_layer_fetch_en[0] && comp_nonzero0;
                comp_fetch_stage[0] <= 1'b0;
                comp_fetch_far_stage[0] <= comp_layer_fetch_en[0] && comp_nonzero0;
                comp_fetch_deep_stage[0] <= 1'b0;
                comp_word_cache_target_x[comp_word_cache_fetch_idx0] <= gp_comp_fetch_target_x;
                comp_word_cache_target_y[comp_word_cache_fetch_idx0] <= gp_comp_vcnt;
                comp_word_cache_valid[comp_word_cache_fetch_idx0] <= 1'b0;
                comp_word_cache_ready[comp_word_cache_fetch_idx0] <=
                    !(comp_layer_fetch_en[0] && comp_nonzero0);
            end
            if (gp1_l0_word_start) begin
                comp_fetch_addr_lo[3] <= comp_addr_lo3;
                comp_fetch_addr_hi[3] <= comp_addr_hi3;
                comp_fetch_target_x[3] <= gp_comp_fetch_target_x;
                comp_fetch_target_y[3] <= gp_comp_vcnt;
                comp_fetch_color[3] <= comp_attr_eff3[6:0];
                comp_fetch_pri[3] <= comp_pri3;
                comp_fetch_pending[3] <= comp_layer_fetch_en[3] && comp_nonzero3;
                comp_fetch_stage[3] <= 1'b0;
                comp_fetch_far_stage[3] <= comp_layer_fetch_en[3] && comp_nonzero3;
                comp_fetch_deep_stage[3] <= 1'b0;
                comp_word_cache_target_x[comp_word_cache_fetch_idx3] <= gp_comp_fetch_target_x;
                comp_word_cache_target_y[comp_word_cache_fetch_idx3] <= gp_comp_vcnt;
                comp_word_cache_valid[comp_word_cache_fetch_idx3] <= 1'b0;
                comp_word_cache_ready[comp_word_cache_fetch_idx3] <=
                    !(comp_layer_fetch_en[3] && comp_nonzero3);
            end
        end

        if (clkdiv == 4'd9) begin
            if (gp0_l1_word_start) begin
                comp_fetch_addr_lo[1] <= comp_addr_lo1;
                comp_fetch_addr_hi[1] <= comp_addr_hi1;
                comp_fetch_target_x[1] <= gp_comp_fetch_target_x;
                comp_fetch_target_y[1] <= gp_comp_vcnt;
                comp_fetch_color[1] <= comp_attr_eff1[6:0];
                comp_fetch_pri[1] <= comp_pri1;
                comp_fetch_pending[1] <= comp_layer_fetch_en[1] && comp_nonzero1;
                comp_fetch_stage[1] <= 1'b0;
                comp_fetch_far_stage[1] <= comp_layer_fetch_en[1] && comp_nonzero1;
                comp_fetch_deep_stage[1] <= 1'b0;
                comp_word_cache_target_x[comp_word_cache_fetch_idx1] <= gp_comp_fetch_target_x;
                comp_word_cache_target_y[comp_word_cache_fetch_idx1] <= gp_comp_vcnt;
                comp_word_cache_valid[comp_word_cache_fetch_idx1] <= 1'b0;
                comp_word_cache_ready[comp_word_cache_fetch_idx1] <=
                    !(comp_layer_fetch_en[1] && comp_nonzero1);
            end
            if (gp1_l1_word_start) begin
                comp_fetch_addr_lo[4] <= comp_addr_lo4;
                comp_fetch_addr_hi[4] <= comp_addr_hi4;
                comp_fetch_target_x[4] <= gp_comp_fetch_target_x;
                comp_fetch_target_y[4] <= gp_comp_vcnt;
                comp_fetch_color[4] <= comp_attr_eff4[6:0];
                comp_fetch_pri[4] <= comp_pri4;
                comp_fetch_pending[4] <= comp_layer_fetch_en[4] && comp_nonzero4;
                comp_fetch_stage[4] <= 1'b0;
                comp_fetch_far_stage[4] <= comp_layer_fetch_en[4] && comp_nonzero4;
                comp_fetch_deep_stage[4] <= 1'b0;
                comp_word_cache_target_x[comp_word_cache_fetch_idx4] <= gp_comp_fetch_target_x;
                comp_word_cache_target_y[comp_word_cache_fetch_idx4] <= gp_comp_vcnt;
                comp_word_cache_valid[comp_word_cache_fetch_idx4] <= 1'b0;
                comp_word_cache_ready[comp_word_cache_fetch_idx4] <=
                    !(comp_layer_fetch_en[4] && comp_nonzero4);
            end
        end

        if ((clkdiv == 4'd13) &&
            !(COMP_BANKED_SCROLL_DIAG && debug_frame_tick)) begin
            if (gp0_l2_word_start) begin
                comp_fetch_addr_lo[2] <= comp_addr_lo2;
                comp_fetch_addr_hi[2] <= comp_addr_hi2;
                comp_fetch_target_x[2] <= gp_comp_fetch_target_x;
                comp_fetch_target_y[2] <= gp_comp_vcnt;
                comp_fetch_color[2] <= comp_attr_eff2[6:0];
                comp_fetch_pri[2] <= comp_pri2;
                comp_fetch_pending[2] <= comp_layer_fetch_en[2] && comp_nonzero2;
                comp_fetch_stage[2] <= 1'b0;
                comp_fetch_far_stage[2] <= comp_layer_fetch_en[2] && comp_nonzero2;
                comp_fetch_deep_stage[2] <= 1'b0;
                comp_word_cache_target_x[comp_word_cache_fetch_idx2] <= gp_comp_fetch_target_x;
                comp_word_cache_target_y[comp_word_cache_fetch_idx2] <= gp_comp_vcnt;
                comp_word_cache_valid[comp_word_cache_fetch_idx2] <= 1'b0;
                comp_word_cache_ready[comp_word_cache_fetch_idx2] <=
                    !(comp_layer_fetch_en[2] && comp_nonzero2);
            end
            if (gp1_l2_word_start) begin
                comp_fetch_addr_lo[5] <= comp_addr_lo5;
                comp_fetch_addr_hi[5] <= comp_addr_hi5;
                comp_fetch_target_x[5] <= gp_comp_fetch_target_x;
                comp_fetch_target_y[5] <= gp_comp_vcnt;
                comp_fetch_color[5] <= comp_attr_eff5[6:0];
                comp_fetch_pri[5] <= comp_pri5;
                comp_fetch_pending[5] <= comp_layer_fetch_en[5] && comp_nonzero5;
                comp_fetch_stage[5] <= 1'b0;
                comp_fetch_far_stage[5] <= comp_layer_fetch_en[5] && comp_nonzero5;
                comp_fetch_deep_stage[5] <= 1'b0;
                comp_word_cache_target_x[comp_word_cache_fetch_idx5] <= gp_comp_fetch_target_x;
                comp_word_cache_target_y[comp_word_cache_fetch_idx5] <= gp_comp_vcnt;
                comp_word_cache_valid[comp_word_cache_fetch_idx5] <= 1'b0;
                comp_word_cache_ready[comp_word_cache_fetch_idx5] <=
                    !(comp_layer_fetch_en[5] && comp_nonzero5);
            end
        end
    end
end

always @(posedge clk) begin
    if (video_runtime_reset) begin
        gp_render_scan_addr <= 13'h0000;
        gp_tile_attr <= 16'h0000;
        gp_tile_code <= 16'h0000;
        pal_scan_addr <= 12'h000;
    end else begin
        if (gp_tile_hex_mode) begin
            if (clkdiv == 4'd0) begin
                gp_render_scan_addr <= gp_scan_tile_attr_addr;
            end else if (clkdiv == 4'd2) begin
                gp_tile_attr <= gp_render_scan_dout;
                gp_render_scan_addr <= gp_scan_tile_code_addr;
            end else if (clkdiv == 4'd4) begin
                gp_tile_code <= gp_render_scan_dout;
            end
        end

        if (clkdiv == 4'd8) begin
            pal_scan_addr <= {1'b0, gp_pixel_lut_addr};
        end
    end
end

always @(posedge clk) begin
    if (cpu_reset) begin
        gp_bus_started <= 1'b0;
        gp_bus_done <= 1'b0;
    end else if (!cpu_bus_active || !cpu_gp_cs) begin
        gp_bus_started <= 1'b0;
        gp_bus_done <= 1'b0;
    end else begin
        if (gp_start) gp_bus_started <= 1'b1;
        if (gp0_done || gp1_done) gp_bus_done <= 1'b1;
    end
end

always @(posedge clk) begin
    if (cpu_reset_base) begin
        v25_stub_addr <= 15'h0000;
        v25_stub_done <= 1'b0;
    end else if (!v25_stub_done) begin
        if (v25_stub_addr == 15'h7fff) begin
            v25_stub_done <= 1'b1;
        end else begin
            v25_stub_addr <= v25_stub_addr + 15'd1;
        end
    end
end

// 94.5 MHz * 32 / 189 = exactly 16 MHz on average. Every V25 state
// machine uses this synchronous enable, so no fabric-derived clock is needed.
always @(posedge clk96) begin
    if (rst96 || dwnld_busy || !v25_stub_done) begin
        v25_cen_accum <= 8'd0;
        v25_cen <= 1'b0;
    end else if (v25_cen_accum >= 8'd157) begin
        v25_cen_accum <= v25_cen_accum + 8'd32 - 8'd189;
        v25_cen <= 1'b1;
    end else begin
        v25_cen_accum <= v25_cen_accum + 8'd32;
        v25_cen <= 1'b0;
    end
end

always @(posedge clk) begin
    if (cpu_reset_base) begin
        v25_sound_released <= 1'b0;
    end else if (ss_restore_commit) begin
        v25_sound_released <= ss_restore_sound_released;
    end else if (cpu_sound_reset_write) begin
        v25_sound_released <= cpu_dout[5];
    end
end

always @(posedge clk) begin
    if (cpu_reset_base || !v25_stub_done) begin
        cpu_ack_seen <= 1'b0;
        cpu_bus_count <= 16'h0000;
        cpu_rom_count <= 8'h00;
        cpu_wram_count <= 8'h00;
        cpu_shared_count <= 8'h00;
        cpu_gp_count <= 8'h00;
        cpu_pal_count <= 8'h00;
        cpu_io_count <= 8'h00;
        cpu_unmap_count <= 8'h00;
        cpu_last_addr <= 24'h000000;
        cpu_last_dout <= 16'h0000;
        cpu_last_din <= 16'h0000;
        ever_cpu_shared <= 1'b0;
        ever_cpu_gp <= 1'b0;
        ever_cpu_pal <= 1'b0;
        ever_cpu_io <= 1'b0;
        ever_cpu_unmap <= 1'b0;
        cpu_shared_read_count <= 8'h00;
        cpu_shared_write_count <= 8'h00;
        cpu_last_shared_addr <= 16'h0000;
        cpu_last_shared_dout <= 8'h00;
        cpu_last_shared_din <= 8'h00;
        cpu_last_gp_addr <= 16'h0000;
        cpu_last_gp_dout <= 16'h0000;
        cpu_last_pal_dout <= 16'h0000;
        fault_snapshot_seen <= 1'b0;
        fault_gp_addr <= 16'h0000;
        fault_gp_data <= 16'h0000;
        fault_gp0_latch <= 16'h0000;
        fault_gp1_latch <= 16'h0000;
        fault_bus_counts <= 16'h0000;
        gp_data_test_seen <= 1'b0;
        cpu_irq4 <= 1'b0;
        ever_cpu_irq4 <= 1'b0;
        ever_cpu_iack <= 1'b0;
        ever_sound_reset <= 1'b0;
        ever_sound_release <= 1'b0;
        fatal_fetch_seen <= 1'b0;
        rom_fetch_0 <= 24'h000000;
        rom_fetch_1 <= 24'h000000;
        rom_fetch_2 <= 24'h000000;
        rom_fetch_3 <= 24'h000000;
        cpu_last_sound_reset <= 8'h00;
    end else begin
        if (ss_restore_commit) begin
            cpu_irq4 <= ss_restore_irq4;
        end else begin
            if (cpu_irq4_start) begin
                cpu_irq4 <= 1'b1;
                ever_cpu_irq4 <= 1'b1;
            end
            if (gp0_irq_clear || gp1_irq_clear) begin
                cpu_irq4 <= 1'b0;
            end
        end
        if (cpu_iack) ever_cpu_iack <= 1'b1;

        if (!cpu_bus_active) begin
            cpu_ack_seen <= 1'b0;
        end else if (cpu_ack_now) begin
            cpu_ack_seen <= 1'b1;
            cpu_bus_count <= cpu_bus_count + 16'd1;
            cpu_last_addr <= cpu_addr8;
            cpu_last_dout <= cpu_dout;
            cpu_last_din <= cpu_din;

            if (!fatal_fetch_seen && cpu_fatal_fetch) begin
                fatal_fetch_seen <= 1'b1;
            end else if (!fatal_fetch_seen && cpu_nonfatal_rom_fetch) begin
                rom_fetch_3 <= rom_fetch_2;
                rom_fetch_2 <= rom_fetch_1;
                rom_fetch_1 <= rom_fetch_0;
                rom_fetch_0 <= cpu_addr8;
            end

            if (cpu_gp_cs && cpu_gp_data_port) gp_data_test_seen <= 1'b1;

            if (!fault_snapshot_seen && gp_data_test_seen && cpu_rom_ack &&
                (cpu_addr8 >= 24'h03bd0a) && (cpu_addr8 < 24'h03bd20)) begin
                fault_snapshot_seen <= 1'b1;
                fault_gp_addr <= cpu_last_gp_addr;
                fault_gp_data <= cpu_last_gp_dout;
                fault_gp0_latch <= gp0_dbg_last_dout;
                fault_gp1_latch <= gp1_dbg_last_dout;
                fault_bus_counts <= {cpu_gp_count, cpu_pal_count};
            end

            if (cpu_rom_bus_cs) cpu_rom_count <= cpu_rom_count + 8'd1;
            if (cpu_wram_cs) cpu_wram_count <= cpu_wram_count + 8'd1;
            if (cpu_shared_cs) cpu_shared_count <= cpu_shared_count + 8'd1;
            if (cpu_gp_cs) cpu_gp_count <= cpu_gp_count + 8'd1;
            if (cpu_pal_cs) cpu_pal_count <= cpu_pal_count + 8'd1;
            if (cpu_io_cs) cpu_io_count <= cpu_io_count + 8'd1;
            if (cpu_unmap_cs) cpu_unmap_count <= cpu_unmap_count + 8'd1;
            if (cpu_shared_read) cpu_shared_read_count <= cpu_shared_read_count + 8'd1;
            if (cpu_shared_we) cpu_shared_write_count <= cpu_shared_write_count + 8'd1;

            if (cpu_shared_cs) ever_cpu_shared <= 1'b1;
            if (cpu_gp_cs) ever_cpu_gp <= 1'b1;
            if (cpu_pal_cs) ever_cpu_pal <= 1'b1;
            if (cpu_io_cs) ever_cpu_io <= 1'b1;
            if (cpu_unmap_cs) ever_cpu_unmap <= 1'b1;

            if (cpu_shared_cs) begin
                cpu_last_shared_addr <= cpu_addr8[15:0];
                cpu_last_shared_dout <= cpu_shared_din;
                cpu_last_shared_din <= cpu_shared_q;
            end
            if (cpu_gp_cs) begin
                cpu_last_gp_addr <= {cpu_gp_read, cpu_gp1_cs, cpu_addr8[13:0]};
                cpu_last_gp_dout <= cpu_gp_read ? (16'h8000 | cpu_gp_data) : cpu_dout;
            end
            if (cpu_pal_write) begin
                cpu_last_pal_dout <= cpu_dout;
            end
            if (cpu_sound_reset_cs && cpu_write) begin
                ever_sound_reset <= 1'b1;
                cpu_last_sound_reset <= cpu_dout[7:0];
                if (cpu_dout[5]) ever_sound_release <= 1'b1;
            end
        end
    end
end

always @(posedge clk) begin
    pxl_cen <= 1'b0;
    pxl2_cen <= 1'b0;

    if (rst) begin
        red <= 8'h00;
        green <= 8'h00;
        blue <= 8'h00;
        LHBL <= 1'b0;
        LVBL <= 1'b0;
    end

    begin
        pxl2_cen <= 1'b0;
        clkdiv <= clkdiv + 4'd1;

        if (clkdiv == 4'd6) begin
            pxl2_cen <= 1'b1;
        end

        if (clkdiv == 4'd13) begin
            clkdiv <= 4'd0;
            pxl_cen <= 1'b1;
            pxl2_cen <= 1'b1;
            if (!rst) begin
                LHBL <= render_lhbl;
                LVBL <= render_lvbl;
            end

            if (hcnt == H_TOTAL-1) begin
                hcnt <= 10'd0;
                if (vcnt == V_TOTAL-1) vcnt <= 9'd0;
                else vcnt <= vcnt + 9'd1;
            end else begin
                hcnt <= hcnt + 10'd1;
            end

            HS <= ~((hcnt >= 10'd340) && (hcnt < 10'd376));
            VS <= ~((vcnt >= 9'd244) && (vcnt < 9'd248));

            if (render_lhbl && render_lvbl) begin
                if (debug_object_status_area) begin
                    case (debug_object_status_col)
                        4'd0: begin
                            red <= (gp0_obj_count != 7'd0) ? 8'hff : 8'h18;
                            green <= 8'h00;
                            blue <= 8'h00;
                        end
                        4'd1: begin
                            red <= 8'h00;
                            green <= (gp1_obj_count != 7'd0) ? 8'hff : 8'h18;
                            blue <= 8'h00;
                        end
                        4'd2: begin
                            red <= 8'h00;
                            green <= 8'h00;
                            blue <= (obj_scan_phase == 4'd10) ? 8'hff : 8'h18;
                        end
                        4'd3: begin
                            red <= (debug_object_line_en ? obj_line_req_pending :
                                    obj_gfx_req_pending) ? 8'hff : 8'h18;
                            green <= (debug_object_line_en ? obj_line_req_pending :
                                      obj_gfx_req_pending) ? 8'hff : 8'h18;
                            blue <= (debug_object_line_en ? obj_line_req_pending :
                                     obj_gfx_req_pending) ? 8'hff : 8'h18;
                        end
                        4'd4: begin
                            red <= 8'h00;
                            green <= (debug_object_line_en ? obj_line_seen_ok :
                                      obj_gfx_seen_ok) ? 8'hff : 8'h18;
                            blue <= (debug_object_line_en ? obj_line_seen_ok :
                                     obj_gfx_seen_ok) ? 8'hff : 8'h18;
                        end
                        4'd5: begin
                            red <= (debug_object_line_en ? obj_line_latched_valid :
                                    (obj_gfx_valid_count != 13'd0)) ? 8'hff : 8'h18;
                            green <= 8'h00;
                            blue <= (debug_object_line_en ? obj_line_latched_valid :
                                     (obj_gfx_valid_count != 13'd0)) ? 8'hff : 8'h18;
                        end
                        4'd6: begin
                            red <= (debug_object_line_en ? obj_line_seen_nonzero :
                                    obj_gfx_seen_nonzero) ? 8'hff : 8'h18;
                            green <= (debug_object_line_en ? obj_line_seen_nonzero :
                                      obj_gfx_seen_nonzero) ? 8'hff : 8'h18;
                            blue <= 8'h00;
                        end
                        4'd7: begin
                            red <= (debug_object_line_en ? obj_line_pick_valid :
                                    obj_gfx_fetch_done) ? 8'hff : 8'h18;
                            green <= (debug_object_line_en ? obj_line_pick_valid :
                                      obj_gfx_fetch_done) ? 8'hff : 8'h18;
                            blue <= (debug_object_line_en ? obj_line_pick_valid :
                                     obj_gfx_fetch_done) ? 8'hff : 8'h18;
                        end
                        default: begin
                            red <= 8'h00;
                            green <= 8'h00;
                            blue <= 8'h00;
                        end
                    endcase
                end else if (debug_object_meta_hex_bit) begin
                    red   <= 8'hff;
                    green <= 8'hff;
                    blue  <= 8'hff;
                end else if (debug_object_meta_area) begin
                    red   <= 8'h00;
                    green <= 8'h00;
                    blue  <= 8'h18;
                end else if (debug_object_tile_area) begin
                    if (debug_object_tile_sample != 4'h0) begin
                        case (debug_object_tile_decode_mode)
                            2'd0: begin
                                red <= 8'hff;
                                green <= {debug_object_tile_sample, debug_object_tile_sample};
                                blue <= 8'h00;
                            end
                            2'd1: begin
                                red <= 8'h00;
                                green <= 8'hff;
                                blue <= {debug_object_tile_sample, debug_object_tile_sample};
                            end
                            2'd2: begin
                                red <= 8'h00;
                                green <= {debug_object_tile_sample, debug_object_tile_sample};
                                blue <= 8'hff;
                            end
                            default: begin
                                red <= 8'hff;
                                green <= {debug_object_tile_sample, debug_object_tile_sample};
                                blue <= 8'hff;
                            end
                        endcase
                    end else if (debug_object_tile_valid) begin
                        red <= 8'h00;
                        green <= 8'h20;
                        blue <= 8'h40;
                    end else begin
                        red <= 8'h20;
                        green <= 8'h00;
                        blue <= 8'h00;
                    end
                end else if (debug_object_box_bit) begin
                    red   <= gp0_obj_box_bit ? 8'hff : 8'h00;
                    green <= gp1_obj_box_bit ? 8'hff : 8'h00;
                    blue  <= 8'hff;
                end else if (debug_object_sprite_bit) begin
                    if (debug_object_line_en) begin
                        red   <= obj_line_sprite_gp_sel ? 8'h00 :
                                 {obj_line_sprite_sample, obj_line_sprite_sample};
                        green <= obj_line_sprite_gp_sel ?
                                 {obj_line_sprite_sample, obj_line_sprite_sample} : 8'h00;
                        blue  <= {obj_line_sprite_sample, obj_line_sprite_sample};
                    end else begin
                        red   <= (gp0_obj_sprite_sample != 4'h0) ?
                                 {gp0_obj_sprite_sample, gp0_obj_sprite_sample} : 8'h00;
                        green <= (gp1_obj_sprite_sample != 4'h0) ?
                                 {gp1_obj_sprite_sample, gp1_obj_sprite_sample} : 8'h00;
                        blue  <= {((gp0_obj_sprite_sample | gp1_obj_sprite_sample) != 4'h0) ?
                                  (gp0_obj_sprite_sample | gp1_obj_sprite_sample) : 4'h0, 4'hf};
                    end
                end else if (debug_object_raw_bit) begin
                    if (debug_object_line_en) begin
                        red   <= obj_line_sprite_gp_sel ? 8'h00 : 8'hff;
                        green <= 8'ha0;
                        blue  <= obj_line_sprite_gp_sel ? 8'hff : 8'h20;
                    end else begin
                        red   <= gp0_obj_sprite_raw_bit ? 8'hff : 8'h00;
                        green <= 8'ha0;
                        blue  <= gp1_obj_sprite_raw_bit ? 8'hff : 8'h20;
                    end
                end else if (debug_object_fill_bit) begin
                    red   <= gp0_obj_fill_bit ? 8'ha0 : 8'h00;
                    green <= gp1_obj_fill_bit ? 8'ha0 : 8'h00;
                    blue  <= 8'ha0;
                end else if (debug_edge_src_en && gp_tile_area) begin
                    case (gp_edge_src_code)
                        4'd0: begin red <= 8'hff; green <= 8'h00; blue <= 8'h00; end
                        4'd1: begin red <= 8'h00; green <= 8'hff; blue <= 8'h00; end
                        4'd2: begin red <= 8'h00; green <= 8'h40; blue <= 8'hff; end
                        4'd3: begin red <= 8'h00; green <= 8'hff; blue <= 8'hff; end
                        4'd4: begin red <= 8'hff; green <= 8'h00; blue <= 8'hff; end
                        4'd5: begin red <= 8'hff; green <= 8'hff; blue <= 8'h00; end
                        4'd6: begin red <= 8'hff; green <= 8'h80; blue <= 8'h00; end
                        4'd7: begin red <= 8'hff; green <= 8'hff; blue <= 8'hff; end
                        default: begin red <= 8'h00; green <= 8'h00; blue <= 8'h00; end
                    endcase
                    if (!gp_edge_src_opaque) begin
                        red <= 8'h00;
                        green <= 8'h00;
                        blue <= 8'h00;
                    end
                end else if (debug_pressure_hex_bit) begin
                    red   <= 8'hff;
                    green <= 8'hf0;
                    blue  <= 8'h80;
                end else if (debug_pressure_area) begin
                    red   <= 8'h08;
                    green <= 8'h10;
                    blue  <= 8'h28;
                end else if (debug_text_overlay_en && !gp_tile_hex_mode && debug_hex_bit) begin
                    red   <= 8'hff;
                    green <= 8'hff;
                    blue  <= 8'hff;
                end else if (debug_text_overlay_en && !gp_tile_hex_mode && debug_hex_row_line) begin
                    red   <= 8'h00;
                    green <= 8'h30;
                    blue  <= 8'hff;
                end else if (debug_text_overlay_en && !gp_tile_hex_mode && rom_status_area) begin
                    red   <= 8'h00;
                    green <= 8'hff;
                    blue  <= 8'h00;
                end else if (gp_rom_probe_hex_bit) begin
                    red   <= 8'hff;
                    green <= 8'hff;
                    blue  <= 8'hff;
                end else if (gp_rom_probe_row_valid) begin
                    red   <= gp_rom_probe_seen ? (gp_rom_probe_match ? 8'h00 : 8'h40) : 8'h00;
                    green <= gp_rom_probe_seen ? (gp_rom_probe_match ? 8'h30 : 8'h00) : 8'h18;
                    blue  <= gp_rom_probe_seen ? 8'h00 : 8'h30;
                end else if (gp_tile_hex_bit) begin
                    red   <= 8'hff;
                    green <= 8'hff;
                    blue  <= 8'hff;
                end else if (gp_tile_hex_code_mode && gp_tile_nonzero) begin
                    red   <= 8'h08;
                    green <= 8'h08;
                    blue  <= 8'h18;
                end else if (gp_pixel_opaque) begin
                    red   <= gp_pal_red;
                    green <= gp_pal_green;
                    blue  <= gp_pal_blue;
                end else begin
                    red   <= 8'h00;
                    green <= 8'h00;
                    blue  <= 8'h00;
                end
            end else begin
                red <= 8'h00;
                green <= 8'h00;
                blue <= 8'h00;
            end
        end

    end
end

endmodule

module batsugun_obj_cache_ram #(
    parameter integer ADDR_W = 7,
    parameter integer DATA_W = 68,
    parameter integer DEPTH = 128
) (
    input                       clk,
    input      [ADDR_W-1:0]     addr,
    input      [DATA_W-1:0]     wr_data,
    input                       wr_en,
    output     [DATA_W-1:0]     rd_data
);

`ifdef SIMULATION
reg [DATA_W-1:0] mem [0:DEPTH-1];
reg [ADDR_W-1:0] rd_addr_reg = {ADDR_W{1'b0}};
reg [DATA_W-1:0] rd_data_reg = {DATA_W{1'b0}};
integer init_i;

initial begin
    for (init_i = 0; init_i < DEPTH; init_i = init_i + 1) begin
        mem[init_i] = {DATA_W{1'b0}};
    end
end

always @(posedge clk) begin
    rd_addr_reg <= addr;
    rd_data_reg <= mem[rd_addr_reg];
    if (wr_en) begin
        mem[addr] <= wr_data;
    end
end

assign rd_data = rd_data_reg;
`else
altsyncram obj_cache_mem (
    .clock0          (clk),
    .address_a       (addr),
    .data_a          (wr_data),
    .wren_a          (wr_en),
    .q_a             (rd_data),
    .aclr0           (1'b0),
    .addressstall_a  (1'b0),
    .byteena_a       (1'b1),
    .clocken0        (1'b1),
    .clocken1        (1'b1),
    .clocken2        (1'b1),
    .clocken3        (1'b1),
    .eccstatus       (),
    .rden_a          (1'b1)
);
defparam
    obj_cache_mem.numwords_a = DEPTH,
    obj_cache_mem.widthad_a = ADDR_W,
    obj_cache_mem.width_a = DATA_W,
    obj_cache_mem.clock_enable_input_a = "BYPASS",
    obj_cache_mem.clock_enable_output_a = "BYPASS",
    obj_cache_mem.intended_device_family = "Cyclone V",
    obj_cache_mem.lpm_type = "altsyncram",
    obj_cache_mem.operation_mode = "SINGLE_PORT",
    obj_cache_mem.outdata_aclr_a = "NONE",
    obj_cache_mem.outdata_reg_a = "CLOCK0",
    obj_cache_mem.power_up_uninitialized = "FALSE",
    obj_cache_mem.read_during_write_mode_port_a = "NEW_DATA_NO_NBE_READ",
    obj_cache_mem.width_byteena_a = 1;
`endif

endmodule

module batsugun_obj_line_ram #(
    parameter integer ADDR_W = 14,
    parameter integer DEPTH = 16384
) (
    input             clk,
    input      [ADDR_W-1:0] build_addr,
    input      [19:0] build_data,
    input             build_we,
    output     [19:0] build_q,
    input      [ADDR_W-1:0] display_addr,
    output     [19:0] display_q
);

`ifdef SIMULATION
reg [19:0] mem [0:DEPTH-1];
reg [19:0] build_q_reg = 20'h00000;
reg [19:0] display_q_reg = 20'h00000;
integer init_i;

initial begin
    for (init_i = 0; init_i < DEPTH; init_i = init_i + 1) begin
        mem[init_i] = 20'h00000;
    end
end

always @(posedge clk) begin
    build_q_reg <= build_we ? build_data : mem[build_addr];
    display_q_reg <= mem[display_addr];
    if (build_we) begin
        mem[build_addr] <= build_data;
    end
end

assign build_q = build_q_reg;
assign display_q = display_q_reg;
`else
altsyncram obj_line_mem (
    .clock0          (clk),
    .address_a       (build_addr),
    .data_a          (build_data),
    .wren_a          (build_we),
    .q_a             (build_q),
    .clock1          (clk),
    .address_b       (display_addr),
    .data_b          (20'h00000),
    .wren_b          (1'b0),
    .q_b             (display_q),
    .aclr0           (1'b0),
    .aclr1           (1'b0),
    .addressstall_a  (1'b0),
    .addressstall_b  (1'b0),
    .byteena_a       (1'b1),
    .byteena_b       (1'b1),
    .clocken0        (1'b1),
    .clocken1        (1'b1),
    .clocken2        (1'b1),
    .clocken3        (1'b1),
    .eccstatus       (),
    .rden_a          (1'b1),
    .rden_b          (1'b1)
);
defparam
    obj_line_mem.numwords_a = DEPTH,
    obj_line_mem.widthad_a = ADDR_W,
    obj_line_mem.width_a = 20,
    obj_line_mem.numwords_b = DEPTH,
    obj_line_mem.widthad_b = ADDR_W,
    obj_line_mem.width_b = 20,
    obj_line_mem.address_reg_b = "CLOCK1",
    obj_line_mem.clock_enable_input_a = "BYPASS",
    obj_line_mem.clock_enable_input_b = "BYPASS",
    obj_line_mem.clock_enable_output_a = "BYPASS",
    obj_line_mem.clock_enable_output_b = "BYPASS",
    obj_line_mem.indata_reg_b = "CLOCK1",
    obj_line_mem.intended_device_family = "Cyclone V",
    obj_line_mem.lpm_type = "altsyncram",
    obj_line_mem.operation_mode = "BIDIR_DUAL_PORT",
    obj_line_mem.outdata_aclr_a = "NONE",
    obj_line_mem.outdata_aclr_b = "NONE",
    obj_line_mem.outdata_reg_a = "UNREGISTERED",
    obj_line_mem.outdata_reg_b = "UNREGISTERED",
    obj_line_mem.power_up_uninitialized = "FALSE",
    obj_line_mem.read_during_write_mode_port_a = "NEW_DATA_NO_NBE_READ",
    obj_line_mem.read_during_write_mode_port_b = "NEW_DATA_NO_NBE_READ",
    obj_line_mem.width_byteena_a = 1,
    obj_line_mem.width_byteena_b = 1,
    obj_line_mem.wrcontrol_wraddress_reg_b = "CLOCK1";
`endif

endmodule
