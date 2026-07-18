module batsugun_video_profiler (
    input  logic         clk,
    input  logic         rst,
    input  logic         enable,
    input  logic         clear_toggle,
    input  logic         snapshot_toggle,
    input  logic [2:0]   page_sel,

    input  logic         frame_edge,
    input  logic         visible,
    input  logic [5:0]   enqueue_mask,
    input  logic [5:0]   overwrite_mask,
    input  logic         primary_launch,
    input  logic         probe_launch,
    input  logic         primary_complete,
    input  logic         probe_complete,
    input  logic         primary_deadline,
    input  logic         probe_deadline,
    input  logic [1:0]   stage_collision,
    input  logic [5:0]   word_miss_mask,
    input  logic         obj_miss,
    input  logic         gp0_obj_write,
    input  logic         gp1_obj_write,
    input  logic         gp0_obj_copy_busy,
    input  logic         gp1_obj_copy_busy,
    input  logic         gp0_obj_snapshot_miss,
    input  logic         gp1_obj_snapshot_miss,
    input  logic         gp0_scroll_write,
    input  logic         gp1_scroll_write,
    input  logic [1:0]   obj_buffer_mode,
    input  logic         object_trace_enable,
    input  logic [127:0] object_trace_meta,
    input  logic [127:0] object_trace_motion,

    input  logic [5:0]   pending,
    input  logic [5:0]   stage,
    input  logic [5:0]   far_stage,
    input  logic [5:0]   deep_stage,
    input  logic         primary_pending,
    input  logic [2:0]   primary_slot,
    input  logic         primary_phase,
    input  logic         probe_pending,
    input  logic [2:0]   probe_slot,
    input  logic         probe_phase,

    input  logic [9:0]   hcnt,
    input  logic [8:0]   vcnt,
    input  logic [3:0]   clkdiv,
    input  logic [2:0]   slot_cs,
    input  logic [2:0]   slot_ok,
    input  logic         ba1_rd,
    input  logic         ba1_ack,
    input  logic         ba1_rdy,
    input  logic         gfx_reset,
    input  logic         ioctl_rom,
    input  logic         dwnld_busy,
    input  logic         prog_we,
    input  logic         prom_we,
    input  logic         startup_hold,
    input  logic         mem_probe,
    input  logic [15:0]  mem_probe_data,

    output logic [127:0] probe
);

function automatic logic [23:0] sat_add24(
    input logic [23:0] value,
    input logic [3:0]  amount
);
    logic [24:0] sum;
    begin
        sum = {1'b0, value} + {{21{1'b0}}, amount};
        sat_add24 = sum[24] ? 24'hffffff : sum[23:0];
    end
endfunction

function automatic logic [3:0] popcount6(input logic [5:0] value);
    integer i;
    begin
        popcount6 = 4'd0;
        for (i = 0; i < 6; i = i + 1)
            popcount6 = popcount6 + value[i];
    end
endfunction

function automatic logic [15:0] sat_inc16(input logic [15:0] value);
    sat_inc16 = value == 16'hffff ? value : value + 16'd1;
endfunction

function automatic logic [8:0] sat_inc9(input logic [8:0] value);
    sat_inc9 = value == 9'h1ff ? value : value + 9'd1;
endfunction

function automatic logic [5:0] sat_inc6(input logic [5:0] value);
    sat_inc6 = value == 6'h3f ? value : value + 6'd1;
endfunction

function automatic logic [7:0] sat_inc8(input logic [7:0] value);
    sat_inc8 = value == 8'hff ? value : value + 8'd1;
endfunction

logic clear_seen;
logic snapshot_seen;
wire clear_event = clear_toggle != clear_seen;
wire snapshot_event = snapshot_toggle != snapshot_seen;

logic [23:0] frame_count;
logic [23:0] frame_enqueue;
logic [23:0] frame_overwrite;
logic [23:0] frame_word_miss;
logic [23:0] frame_primary_launch;
logic [23:0] frame_probe_launch;
logic [23:0] frame_primary_complete;
logic [23:0] frame_probe_complete;
logic [15:0] frame_primary_deadline;
logic [15:0] frame_probe_deadline;
logic [23:0] frame_stage_collision;
logic [23:0] frame_ba1_req;
logic [23:0] frame_ba1_ack;
logic [23:0] frame_ba1_rdy;
logic [23:0] frame_ba1_wait;
logic [23:0] frame_slot0_ok;
logic [23:0] frame_slot1_ok;
logic [23:0] frame_slot2_ok;
logic [23:0] frame_obj_miss;
logic [23:0] frame_visible_pending;

logic [23:0] last_enqueue;
logic [23:0] last_overwrite;
logic [23:0] last_word_miss;
logic [23:0] last_primary_launch;
logic [23:0] last_probe_launch;
logic [23:0] last_primary_complete;
logic [23:0] last_probe_complete;
logic [15:0] last_primary_deadline;
logic [15:0] last_probe_deadline;
logic [23:0] last_stage_collision;
logic [23:0] last_ba1_req;
logic [23:0] last_ba1_ack;
logic [23:0] last_ba1_rdy;
logic [23:0] last_ba1_wait;
logic [23:0] last_slot0_ok;
logic [23:0] last_slot1_ok;
logic [23:0] last_slot2_ok;
logic [23:0] last_obj_miss;
logic [23:0] last_visible_pending;

logic [8:0]  frame_gp0_obj_writes;
logic [8:0]  frame_gp1_obj_writes;
logic [5:0]  frame_gp0_obj_vblank;
logic [5:0]  frame_gp1_obj_vblank;
logic [5:0]  frame_gp0_obj_overlap;
logic [5:0]  frame_gp1_obj_overlap;
logic        frame_gp0_obj_seen;
logic        frame_gp1_obj_seen;
logic [8:0]  frame_gp0_obj_first_v;
logic [8:0]  frame_gp0_obj_last_v;
logic [8:0]  frame_gp1_obj_first_v;
logic [8:0]  frame_gp1_obj_last_v;
logic [9:0]  frame_gp0_obj_last_h;
logic [9:0]  frame_gp1_obj_last_h;
logic [5:0]  frame_gp0_obj_snapshot_miss;
logic [5:0]  frame_gp1_obj_snapshot_miss;
logic        frame_gp0_scroll_seen;
logic        frame_gp1_scroll_seen;
logic [8:0]  frame_gp0_scroll_last_v;
logic [8:0]  frame_gp1_scroll_last_v;
logic [5:0]  frame_gp0_obj_bins [0:7];
logic [5:0]  frame_gp1_obj_bins [0:7];

logic [8:0]  last_gp0_obj_writes;
logic [8:0]  last_gp1_obj_writes;
logic [5:0]  last_gp0_obj_vblank;
logic [5:0]  last_gp1_obj_vblank;
logic [5:0]  last_gp0_obj_overlap;
logic [5:0]  last_gp1_obj_overlap;
logic        last_gp0_obj_seen;
logic        last_gp1_obj_seen;
logic [8:0]  last_gp0_obj_first_v;
logic [8:0]  last_gp0_obj_last_v;
logic [8:0]  last_gp1_obj_first_v;
logic [8:0]  last_gp1_obj_last_v;
logic [9:0]  last_gp0_obj_last_h;
logic [9:0]  last_gp1_obj_last_h;
logic [5:0]  last_gp0_obj_snapshot_miss;
logic [5:0]  last_gp1_obj_snapshot_miss;
logic        last_gp0_scroll_seen;
logic        last_gp1_scroll_seen;
logic [8:0]  last_gp0_scroll_last_v;
logic [8:0]  last_gp1_scroll_last_v;
logic [5:0]  last_gp0_obj_bins [0:7];
logic [5:0]  last_gp1_obj_bins [0:7];

wire [2:0] obj_timing_bin = (vcnt >= 9'd224) ? 3'd7 : vcnt[7:5];

logic        ba1_rd_l;
logic        ba1_inflight;
logic [15:0] ba1_grant_run;
logic [15:0] ba1_service_run;
logic [15:0] max_ba1_grant;
logic [15:0] max_ba1_service;

logic        first_fault_seen;
logic [2:0]  first_fault_kind;
logic [23:0] first_fault_frame;
logic [9:0]  first_fault_hcnt;
logic [8:0]  first_fault_vcnt;
logic [3:0]  first_fault_clkdiv;
logic [5:0]  first_fault_overwrite;
logic [5:0]  first_fault_word_miss;
logic [5:0]  first_fault_pending;
logic [5:0]  first_fault_stage;
logic [5:0]  first_fault_far;
logic [5:0]  first_fault_deep;
logic [7:0]  first_fault_primary;
logic [7:0]  first_fault_probe;
logic [5:0]  first_fault_slots;
logic [2:0]  first_fault_ba1;

wire fault_eligible = frame_count >= 24'd2;
wire fault_event = fault_eligible &&
    ((|overwrite_mask) || primary_deadline || probe_deadline || (|word_miss_mask));
wire [2:0] fault_kind = (|overwrite_mask) ? 3'd1 :
                        primary_deadline  ? 3'd2 :
                        probe_deadline    ? 3'd3 : 3'd4;

wire [7:0] live_flags = {
    first_fault_seen,
    ba1_inflight,
    ba1_rdy,
    ba1_ack,
    ba1_rd,
    visible,
    enable,
    1'b1
};

wire [127:0] live_page0 = {
    last_word_miss,
    last_overwrite,
    last_enqueue,
    frame_count,
    live_flags,
    8'h01,
    8'h56,
    8'h42
};

wire [127:0] live_page1 = {
    8'h00,
    last_stage_collision,
    last_probe_complete,
    last_primary_complete,
    last_probe_launch,
    last_primary_launch
};

wire [127:0] live_page2 = {
    max_ba1_service,
    max_ba1_grant,
    last_ba1_wait,
    last_ba1_rdy,
    last_ba1_ack,
    last_ba1_req
};

wire [127:0] live_page3 = {
    8'h00,
    last_visible_pending,
    last_obj_miss,
    last_slot2_ok,
    last_slot1_ok,
    last_slot0_ok
};

wire [127:0] live_page4_standard = {
    16'h0000,
    first_fault_frame,
    first_fault_ba1,
    first_fault_slots,
    first_fault_probe,
    first_fault_primary,
    first_fault_deep,
    first_fault_far,
    first_fault_stage,
    first_fault_pending,
    first_fault_word_miss,
    first_fault_overwrite,
    first_fault_clkdiv,
    first_fault_vcnt,
    first_fault_hcnt,
    first_fault_kind,
    first_fault_seen
};

wire [127:0] live_page5_standard = {
    {gfx_reset, startup_hold, mem_probe, ioctl_rom, dwnld_busy,
     prog_we, prom_we, slot_ok[1]},
    mem_probe_data,
    14'h0000,
    clkdiv,
    vcnt,
    hcnt,
    {ba1_rdy, ba1_ack, ba1_rd},
    slot_ok,
    slot_cs,
    {probe_pending, probe_slot, probe_phase, 3'b000},
    {primary_pending, primary_slot, primary_phase, 3'b000},
    deep_stage,
    far_stage,
    stage,
    pending,
    word_miss_mask,
    overwrite_mask,
    enqueue_mask
};

wire [127:0] live_page4 = object_trace_enable ?
                          object_trace_meta : live_page4_standard;
wire [127:0] live_page5 = object_trace_enable ?
                          object_trace_motion : live_page5_standard;

wire [127:0] live_page6 = {
    gp1_obj_copy_busy,
    gp0_obj_copy_busy,
    last_gp1_scroll_seen,
    last_gp0_scroll_seen,
    last_gp1_obj_seen,
    last_gp0_obj_seen,
    last_gp1_scroll_last_v,
    last_gp0_scroll_last_v,
    last_gp1_obj_overlap,
    last_gp0_obj_overlap,
    last_gp1_obj_vblank,
    last_gp0_obj_vblank,
    last_gp1_obj_writes,
    last_gp0_obj_writes,
    last_gp1_obj_last_v,
    last_gp1_obj_first_v,
    last_gp0_obj_last_v,
    last_gp0_obj_first_v,
    obj_buffer_mode,
    frame_count
};

wire [127:0] live_page7 = {
    last_gp1_obj_snapshot_miss,
    last_gp0_obj_snapshot_miss,
    last_gp1_obj_last_h,
    last_gp0_obj_last_h,
    last_gp1_obj_bins[7], last_gp1_obj_bins[6],
    last_gp1_obj_bins[5], last_gp1_obj_bins[4],
    last_gp1_obj_bins[3], last_gp1_obj_bins[2],
    last_gp1_obj_bins[1], last_gp1_obj_bins[0],
    last_gp0_obj_bins[7], last_gp0_obj_bins[6],
    last_gp0_obj_bins[5], last_gp0_obj_bins[4],
    last_gp0_obj_bins[3], last_gp0_obj_bins[2],
    last_gp0_obj_bins[1], last_gp0_obj_bins[0]
};

logic [127:0] snapshot_page [0:7];
integer page_i;
integer obj_bin_i;

always_comb begin
    case (page_sel)
        3'd0: probe = snapshot_page[0];
        3'd1: probe = snapshot_page[1];
        3'd2: probe = snapshot_page[2];
        3'd3: probe = snapshot_page[3];
        3'd4: probe = snapshot_page[4];
        3'd5: probe = snapshot_page[5];
        3'd6: probe = snapshot_page[6];
        default: probe = snapshot_page[7];
    endcase
end

always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        clear_seen <= 1'b0;
        snapshot_seen <= 1'b0;
        frame_count <= 24'd0;
        frame_enqueue <= 24'd0;
        frame_overwrite <= 24'd0;
        frame_word_miss <= 24'd0;
        frame_primary_launch <= 24'd0;
        frame_probe_launch <= 24'd0;
        frame_primary_complete <= 24'd0;
        frame_probe_complete <= 24'd0;
        frame_primary_deadline <= 16'd0;
        frame_probe_deadline <= 16'd0;
        frame_stage_collision <= 24'd0;
        frame_ba1_req <= 24'd0;
        frame_ba1_ack <= 24'd0;
        frame_ba1_rdy <= 24'd0;
        frame_ba1_wait <= 24'd0;
        frame_slot0_ok <= 24'd0;
        frame_slot1_ok <= 24'd0;
        frame_slot2_ok <= 24'd0;
        frame_obj_miss <= 24'd0;
        frame_visible_pending <= 24'd0;
        last_enqueue <= 24'd0;
        last_overwrite <= 24'd0;
        last_word_miss <= 24'd0;
        last_primary_launch <= 24'd0;
        last_probe_launch <= 24'd0;
        last_primary_complete <= 24'd0;
        last_probe_complete <= 24'd0;
        last_primary_deadline <= 16'd0;
        last_probe_deadline <= 16'd0;
        last_stage_collision <= 24'd0;
        last_ba1_req <= 24'd0;
        last_ba1_ack <= 24'd0;
        last_ba1_rdy <= 24'd0;
        last_ba1_wait <= 24'd0;
        last_slot0_ok <= 24'd0;
        last_slot1_ok <= 24'd0;
        last_slot2_ok <= 24'd0;
        last_obj_miss <= 24'd0;
        last_visible_pending <= 24'd0;
        frame_gp0_obj_writes <= 9'd0;
        frame_gp1_obj_writes <= 9'd0;
        frame_gp0_obj_vblank <= 6'd0;
        frame_gp1_obj_vblank <= 6'd0;
        frame_gp0_obj_overlap <= 6'd0;
        frame_gp1_obj_overlap <= 6'd0;
        frame_gp0_obj_seen <= 1'b0;
        frame_gp1_obj_seen <= 1'b0;
        frame_gp0_obj_first_v <= 9'd0;
        frame_gp0_obj_last_v <= 9'd0;
        frame_gp1_obj_first_v <= 9'd0;
        frame_gp1_obj_last_v <= 9'd0;
        frame_gp0_obj_last_h <= 10'd0;
        frame_gp1_obj_last_h <= 10'd0;
        frame_gp0_obj_snapshot_miss <= 6'd0;
        frame_gp1_obj_snapshot_miss <= 6'd0;
        frame_gp0_scroll_seen <= 1'b0;
        frame_gp1_scroll_seen <= 1'b0;
        frame_gp0_scroll_last_v <= 9'd0;
        frame_gp1_scroll_last_v <= 9'd0;
        last_gp0_obj_writes <= 9'd0;
        last_gp1_obj_writes <= 9'd0;
        last_gp0_obj_vblank <= 6'd0;
        last_gp1_obj_vblank <= 6'd0;
        last_gp0_obj_overlap <= 6'd0;
        last_gp1_obj_overlap <= 6'd0;
        last_gp0_obj_seen <= 1'b0;
        last_gp1_obj_seen <= 1'b0;
        last_gp0_obj_first_v <= 9'd0;
        last_gp0_obj_last_v <= 9'd0;
        last_gp1_obj_first_v <= 9'd0;
        last_gp1_obj_last_v <= 9'd0;
        last_gp0_obj_last_h <= 10'd0;
        last_gp1_obj_last_h <= 10'd0;
        last_gp0_obj_snapshot_miss <= 6'd0;
        last_gp1_obj_snapshot_miss <= 6'd0;
        last_gp0_scroll_seen <= 1'b0;
        last_gp1_scroll_seen <= 1'b0;
        last_gp0_scroll_last_v <= 9'd0;
        last_gp1_scroll_last_v <= 9'd0;
        ba1_rd_l <= 1'b0;
        ba1_inflight <= 1'b0;
        ba1_grant_run <= 16'd0;
        ba1_service_run <= 16'd0;
        max_ba1_grant <= 16'd0;
        max_ba1_service <= 16'd0;
        first_fault_seen <= 1'b0;
        first_fault_kind <= 3'd0;
        first_fault_frame <= 24'd0;
        first_fault_hcnt <= 10'd0;
        first_fault_vcnt <= 9'd0;
        first_fault_clkdiv <= 4'd0;
        first_fault_overwrite <= 6'd0;
        first_fault_word_miss <= 6'd0;
        first_fault_pending <= 6'd0;
        first_fault_stage <= 6'd0;
        first_fault_far <= 6'd0;
        first_fault_deep <= 6'd0;
        first_fault_primary <= 8'd0;
        first_fault_probe <= 8'd0;
        first_fault_slots <= 6'd0;
        first_fault_ba1 <= 3'd0;
        for (obj_bin_i = 0; obj_bin_i < 8; obj_bin_i = obj_bin_i + 1) begin
            frame_gp0_obj_bins[obj_bin_i] <= 6'd0;
            frame_gp1_obj_bins[obj_bin_i] <= 6'd0;
            last_gp0_obj_bins[obj_bin_i] <= 6'd0;
            last_gp1_obj_bins[obj_bin_i] <= 6'd0;
        end
        for (page_i = 0; page_i < 8; page_i = page_i + 1)
            snapshot_page[page_i] <= 128'd0;
    end else begin
        clear_seen <= clear_toggle;
        snapshot_seen <= snapshot_toggle;
        ba1_rd_l <= ba1_rd;

        if (clear_event) begin
            frame_count <= 24'd0;
            frame_enqueue <= 24'd0;
            frame_overwrite <= 24'd0;
            frame_word_miss <= 24'd0;
            frame_primary_launch <= 24'd0;
            frame_probe_launch <= 24'd0;
            frame_primary_complete <= 24'd0;
            frame_probe_complete <= 24'd0;
            frame_primary_deadline <= 16'd0;
            frame_probe_deadline <= 16'd0;
            frame_stage_collision <= 24'd0;
            frame_ba1_req <= 24'd0;
            frame_ba1_ack <= 24'd0;
            frame_ba1_rdy <= 24'd0;
            frame_ba1_wait <= 24'd0;
            frame_slot0_ok <= 24'd0;
            frame_slot1_ok <= 24'd0;
            frame_slot2_ok <= 24'd0;
            frame_obj_miss <= 24'd0;
            frame_visible_pending <= 24'd0;
            frame_gp0_obj_writes <= 9'd0;
            frame_gp1_obj_writes <= 9'd0;
            frame_gp0_obj_vblank <= 6'd0;
            frame_gp1_obj_vblank <= 6'd0;
            frame_gp0_obj_overlap <= 6'd0;
            frame_gp1_obj_overlap <= 6'd0;
            frame_gp0_obj_seen <= 1'b0;
            frame_gp1_obj_seen <= 1'b0;
            frame_gp0_scroll_seen <= 1'b0;
            frame_gp1_scroll_seen <= 1'b0;
            frame_gp0_obj_first_v <= 9'd0;
            frame_gp0_obj_last_v <= 9'd0;
            frame_gp1_obj_first_v <= 9'd0;
            frame_gp1_obj_last_v <= 9'd0;
            frame_gp0_obj_last_h <= 10'd0;
            frame_gp1_obj_last_h <= 10'd0;
            frame_gp0_obj_snapshot_miss <= 6'd0;
            frame_gp1_obj_snapshot_miss <= 6'd0;
            frame_gp0_scroll_last_v <= 9'd0;
            frame_gp1_scroll_last_v <= 9'd0;
            last_gp0_obj_writes <= 9'd0;
            last_gp1_obj_writes <= 9'd0;
            last_gp0_obj_vblank <= 6'd0;
            last_gp1_obj_vblank <= 6'd0;
            last_gp0_obj_overlap <= 6'd0;
            last_gp1_obj_overlap <= 6'd0;
            last_gp0_obj_seen <= 1'b0;
            last_gp1_obj_seen <= 1'b0;
            last_gp0_obj_first_v <= 9'd0;
            last_gp0_obj_last_v <= 9'd0;
            last_gp1_obj_first_v <= 9'd0;
            last_gp1_obj_last_v <= 9'd0;
            last_gp0_obj_last_h <= 10'd0;
            last_gp1_obj_last_h <= 10'd0;
            last_gp0_obj_snapshot_miss <= 6'd0;
            last_gp1_obj_snapshot_miss <= 6'd0;
            last_gp0_scroll_seen <= 1'b0;
            last_gp1_scroll_seen <= 1'b0;
            last_gp0_scroll_last_v <= 9'd0;
            last_gp1_scroll_last_v <= 9'd0;
            ba1_inflight <= 1'b0;
            ba1_grant_run <= 16'd0;
            ba1_service_run <= 16'd0;
            max_ba1_grant <= 16'd0;
            max_ba1_service <= 16'd0;
            first_fault_seen <= 1'b0;
            for (obj_bin_i = 0; obj_bin_i < 8; obj_bin_i = obj_bin_i + 1) begin
                frame_gp0_obj_bins[obj_bin_i] <= 6'd0;
                frame_gp1_obj_bins[obj_bin_i] <= 6'd0;
                last_gp0_obj_bins[obj_bin_i] <= 6'd0;
                last_gp1_obj_bins[obj_bin_i] <= 6'd0;
            end
            for (page_i = 0; page_i < 8; page_i = page_i + 1)
                snapshot_page[page_i] <= 128'd0;
        end else begin
            if (snapshot_event) begin
                snapshot_page[0] <= live_page0;
                snapshot_page[1] <= live_page1;
                snapshot_page[2] <= live_page2;
                snapshot_page[3] <= live_page3;
                snapshot_page[4] <= live_page4;
                snapshot_page[5] <= live_page5;
                snapshot_page[6] <= live_page6;
                snapshot_page[7] <= live_page7;
            end

            if (fault_event && !first_fault_seen) begin
                first_fault_seen <= 1'b1;
                first_fault_kind <= fault_kind;
                first_fault_frame <= frame_count;
                first_fault_hcnt <= hcnt;
                first_fault_vcnt <= vcnt;
                first_fault_clkdiv <= clkdiv;
                first_fault_overwrite <= overwrite_mask;
                first_fault_word_miss <= word_miss_mask;
                first_fault_pending <= pending;
                first_fault_stage <= stage;
                first_fault_far <= far_stage;
                first_fault_deep <= deep_stage;
                first_fault_primary <= {primary_pending, primary_slot,
                                        primary_phase, 3'b000};
                first_fault_probe <= {probe_pending, probe_slot,
                                      probe_phase, 3'b000};
                first_fault_slots <= {slot_ok, slot_cs};
                first_fault_ba1 <= {ba1_rdy, ba1_ack, ba1_rd};
            end

            if (ba1_rd && !ba1_rd_l) begin
                ba1_grant_run <= 16'd1;
            end else if (ba1_rd && !ba1_ack && !ba1_inflight) begin
                ba1_grant_run <= sat_inc16(ba1_grant_run);
            end
            if (ba1_ack) begin
                if (ba1_grant_run > max_ba1_grant)
                    max_ba1_grant <= ba1_grant_run;
                ba1_grant_run <= 16'd0;
                ba1_inflight <= 1'b1;
                ba1_service_run <= 16'd1;
            end else if (ba1_inflight) begin
                ba1_service_run <= sat_inc16(ba1_service_run);
            end
            if (ba1_rdy) begin
                if (ba1_service_run > max_ba1_service)
                    max_ba1_service <= ba1_service_run;
                ba1_inflight <= 1'b0;
                ba1_service_run <= 16'd0;
            end

            if (enable && frame_edge) begin
                frame_count <= sat_add24(frame_count, 4'd1);
                last_enqueue <= frame_enqueue;
                last_overwrite <= frame_overwrite;
                last_word_miss <= frame_word_miss;
                last_primary_launch <= frame_primary_launch;
                last_probe_launch <= frame_probe_launch;
                last_primary_complete <= frame_primary_complete;
                last_probe_complete <= frame_probe_complete;
                last_primary_deadline <= frame_primary_deadline;
                last_probe_deadline <= frame_probe_deadline;
                last_stage_collision <= frame_stage_collision;
                last_ba1_req <= frame_ba1_req;
                last_ba1_ack <= frame_ba1_ack;
                last_ba1_rdy <= frame_ba1_rdy;
                last_ba1_wait <= frame_ba1_wait;
                last_slot0_ok <= frame_slot0_ok;
                last_slot1_ok <= frame_slot1_ok;
                last_slot2_ok <= frame_slot2_ok;
                last_obj_miss <= frame_obj_miss;
                last_visible_pending <= frame_visible_pending;
                last_gp0_obj_writes <= frame_gp0_obj_writes;
                last_gp1_obj_writes <= frame_gp1_obj_writes;
                last_gp0_obj_vblank <= frame_gp0_obj_vblank;
                last_gp1_obj_vblank <= frame_gp1_obj_vblank;
                last_gp0_obj_overlap <= frame_gp0_obj_overlap;
                last_gp1_obj_overlap <= frame_gp1_obj_overlap;
                last_gp0_obj_seen <= frame_gp0_obj_seen;
                last_gp1_obj_seen <= frame_gp1_obj_seen;
                last_gp0_obj_first_v <= frame_gp0_obj_first_v;
                last_gp0_obj_last_v <= frame_gp0_obj_last_v;
                last_gp1_obj_first_v <= frame_gp1_obj_first_v;
                last_gp1_obj_last_v <= frame_gp1_obj_last_v;
                last_gp0_obj_last_h <= frame_gp0_obj_last_h;
                last_gp1_obj_last_h <= frame_gp1_obj_last_h;
                last_gp0_obj_snapshot_miss <= frame_gp0_obj_snapshot_miss;
                last_gp1_obj_snapshot_miss <= frame_gp1_obj_snapshot_miss;
                last_gp0_scroll_seen <= frame_gp0_scroll_seen;
                last_gp1_scroll_seen <= frame_gp1_scroll_seen;
                last_gp0_scroll_last_v <= frame_gp0_scroll_last_v;
                last_gp1_scroll_last_v <= frame_gp1_scroll_last_v;
                frame_enqueue <= 24'd0;
                frame_overwrite <= 24'd0;
                frame_word_miss <= 24'd0;
                frame_primary_launch <= 24'd0;
                frame_probe_launch <= 24'd0;
                frame_primary_complete <= 24'd0;
                frame_probe_complete <= 24'd0;
                frame_primary_deadline <= 16'd0;
                frame_probe_deadline <= 16'd0;
                frame_stage_collision <= 24'd0;
                frame_ba1_req <= 24'd0;
                frame_ba1_ack <= 24'd0;
                frame_ba1_rdy <= 24'd0;
                frame_ba1_wait <= 24'd0;
                frame_slot0_ok <= 24'd0;
                frame_slot1_ok <= 24'd0;
                frame_slot2_ok <= 24'd0;
                frame_obj_miss <= 24'd0;
                frame_visible_pending <= 24'd0;
                frame_gp0_obj_writes <= 9'd0;
                frame_gp1_obj_writes <= 9'd0;
                frame_gp0_obj_vblank <= 6'd0;
                frame_gp1_obj_vblank <= 6'd0;
                frame_gp0_obj_overlap <= 6'd0;
                frame_gp1_obj_overlap <= 6'd0;
                frame_gp0_obj_seen <= 1'b0;
                frame_gp1_obj_seen <= 1'b0;
                frame_gp0_obj_last_h <= 10'd0;
                frame_gp1_obj_last_h <= 10'd0;
                frame_gp0_obj_snapshot_miss <= 6'd0;
                frame_gp1_obj_snapshot_miss <= 6'd0;
                frame_gp0_scroll_seen <= 1'b0;
                frame_gp1_scroll_seen <= 1'b0;
                for (obj_bin_i = 0; obj_bin_i < 8; obj_bin_i = obj_bin_i + 1) begin
                    last_gp0_obj_bins[obj_bin_i] <= frame_gp0_obj_bins[obj_bin_i];
                    last_gp1_obj_bins[obj_bin_i] <= frame_gp1_obj_bins[obj_bin_i];
                    frame_gp0_obj_bins[obj_bin_i] <= 6'd0;
                    frame_gp1_obj_bins[obj_bin_i] <= 6'd0;
                end
            end else if (enable) begin
                frame_enqueue <= sat_add24(frame_enqueue, popcount6(enqueue_mask));
                frame_overwrite <= sat_add24(frame_overwrite, popcount6(overwrite_mask));
                frame_word_miss <= sat_add24(frame_word_miss, popcount6(word_miss_mask));
                frame_primary_launch <= sat_add24(frame_primary_launch,
                                                  {3'd0, primary_launch});
                frame_probe_launch <= sat_add24(frame_probe_launch,
                                                {3'd0, probe_launch});
                frame_primary_complete <= sat_add24(frame_primary_complete,
                                                    {3'd0, primary_complete});
                frame_probe_complete <= sat_add24(frame_probe_complete,
                                                  {3'd0, probe_complete});
                if (primary_deadline)
                    frame_primary_deadline <= sat_inc16(frame_primary_deadline);
                if (probe_deadline)
                    frame_probe_deadline <= sat_inc16(frame_probe_deadline);
                frame_stage_collision <= sat_add24(frame_stage_collision,
                    {2'd0, stage_collision[0]} + {2'd0, stage_collision[1]});
                frame_ba1_req <= sat_add24(frame_ba1_req,
                                           {3'd0, ba1_rd && !ba1_rd_l});
                frame_ba1_ack <= sat_add24(frame_ba1_ack, {3'd0, ba1_ack});
                frame_ba1_rdy <= sat_add24(frame_ba1_rdy, {3'd0, ba1_rdy});
                frame_ba1_wait <= sat_add24(frame_ba1_wait,
                                            {3'd0, ba1_rd && !ba1_ack});
                frame_slot0_ok <= sat_add24(frame_slot0_ok, {3'd0, slot_ok[0]});
                frame_slot1_ok <= sat_add24(frame_slot1_ok, {3'd0, slot_ok[1]});
                frame_slot2_ok <= sat_add24(frame_slot2_ok, {3'd0, slot_ok[2]});
                frame_obj_miss <= sat_add24(frame_obj_miss, {3'd0, obj_miss});
                frame_visible_pending <= sat_add24(frame_visible_pending,
                    {3'd0, visible && (|pending)});
                if (gp0_obj_write) begin
                    frame_gp0_obj_writes <= sat_inc9(frame_gp0_obj_writes);
                    frame_gp0_obj_bins[obj_timing_bin] <=
                        sat_inc6(frame_gp0_obj_bins[obj_timing_bin]);
                    if (!frame_gp0_obj_seen)
                        frame_gp0_obj_first_v <= vcnt;
                    frame_gp0_obj_last_v <= vcnt;
                    frame_gp0_obj_last_h <= hcnt;
                    frame_gp0_obj_seen <= 1'b1;
                    if (vcnt >= 9'd240)
                        frame_gp0_obj_vblank <= sat_inc6(frame_gp0_obj_vblank);
                    if (gp0_obj_copy_busy)
                        frame_gp0_obj_overlap <= sat_inc6(frame_gp0_obj_overlap);
                end
                if (gp1_obj_write) begin
                    frame_gp1_obj_writes <= sat_inc9(frame_gp1_obj_writes);
                    frame_gp1_obj_bins[obj_timing_bin] <=
                        sat_inc6(frame_gp1_obj_bins[obj_timing_bin]);
                    if (!frame_gp1_obj_seen)
                        frame_gp1_obj_first_v <= vcnt;
                    frame_gp1_obj_last_v <= vcnt;
                    frame_gp1_obj_last_h <= hcnt;
                    frame_gp1_obj_seen <= 1'b1;
                    if (vcnt >= 9'd240)
                        frame_gp1_obj_vblank <= sat_inc6(frame_gp1_obj_vblank);
                    if (gp1_obj_copy_busy)
                        frame_gp1_obj_overlap <= sat_inc6(frame_gp1_obj_overlap);
                end
                if (gp0_obj_snapshot_miss)
                    frame_gp0_obj_snapshot_miss <=
                        sat_inc6(frame_gp0_obj_snapshot_miss);
                if (gp1_obj_snapshot_miss)
                    frame_gp1_obj_snapshot_miss <=
                        sat_inc6(frame_gp1_obj_snapshot_miss);
                if (gp0_scroll_write) begin
                    frame_gp0_scroll_seen <= 1'b1;
                    frame_gp0_scroll_last_v <= vcnt;
                end
                if (gp1_scroll_write) begin
                    frame_gp1_scroll_seen <= 1'b1;
                    frame_gp1_scroll_last_v <= vcnt;
                end
            end
        end
    end
end

endmodule
