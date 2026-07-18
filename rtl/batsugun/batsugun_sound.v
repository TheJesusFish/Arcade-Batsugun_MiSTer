// SPDX-License-Identifier: BSD-3-Clause
//
// Batsugun TP-030 sound subsystem. The V25 owns one port of the uploaded
// shared RAM; YM2151 and OKI writes cross into the 94.5 MHz sound domain.

module batsugun_sound (
    input              reset,
    input              clk16,
    input              v25_cen,
    input              clk_sound,
    input              v25_enable,
    input      [7:0]   dip_a,
    input      [7:0]   dip_b,
    input      [7:0]   region,
    input              debug_hold_v25,
    input              debug_block_ym,
    input              debug_block_oki,
    input              ym_enable,
    input              oki_enable,
    input              ss_hold,

    input              ss_restore_enable,
    input              ss_restore_commit,
    input              ss_restore_bgm_valid,
    input      [7:0]   ss_restore_bgm_command,
    input      [7:0]   ss_restore_bgm_argument,
    input      [63:0]  ss_data,
    input      [31:0]  ss_addr,
    input      [7:0]   ss_select,
    input              ss_write,
    input              ss_read,
    input              ss_query,
    output     [63:0]  ss_data_out,
    output             ss_ack,
    output             v25_state_idle,
    output             sound_state_idle,
    output             sound_state_held,

    output     [14:0]  shared_addr,
    output     [7:0]   shared_dout,
    output             shared_we,
    input      [7:0]   shared_din,
    output     [8:0]   boot_shadow_addr,
    input      [7:0]   boot_shadow_data,

    output     [17:0]  oki_rom_addr,
    input      [7:0]   oki_rom_data,
    input              oki_rom_ok,

    output signed [15:0] snd_mono,
    output             sample,

    output             debug_fault,
    output             debug_halted,
    output     [19:0]  debug_pc,
    output reg         debug_ym_write,
    output reg         debug_ym_a0,
    output reg [7:0]   debug_ym_data,
    output reg         debug_oki_write,
    output reg [7:0]   debug_oki_data,
    output reg         debug_cdc_overrun
);

localparam [19:0] YM_ADDR = 20'h00000;
localparam [19:0] YM_DATA = 20'h00001;
localparam [19:0] OKI_DATA = 20'h00004;

wire [19:0] v25_bus_addr;
wire [7:0]  v25_bus_dout;
reg  [7:0]  v25_bus_din;
wire        v25_bus_doe;
wire        v25_bus_r_w;
wire        v25_bus_mreq_n;
wire        v25_bus_mstb_n;
wire        v25_bus_iostb_n;

// Sound is an explicitly restarted save-state domain. The uploaded program
// and reset vectors occupy 7e00-7fff; 7800-7dff is zero-filled V25 work RAM.
// Scrubbing that work area lets the V25 and both sound chips cold-boot against
// restored shared RAM without inheriting a stale command or scheduler state.
assign ss_ack = 1'b0;
assign ss_data_out = 64'd0;

wire sound_ready;
wire sound_reset = reset | ss_restore_commit;
localparam [14:0] RESTORE_SCRUB_FIRST = 15'h7800;
localparam [10:0] RESTORE_SCRUB_LAST = 11'h5ff;
localparam [14:0] RESTORE_BOOT_FIRST = 15'h7e00;
localparam [8:0] RESTORE_BOOT_LAST = 9'h1ff;
localparam [1:0]
    RESTORE_IDLE         = 2'd0,
    RESTORE_CLEAR_WORK   = 2'd1,
    RESTORE_PRIME_SHADOW = 2'd2,
    RESTORE_WRITE_BOOT   = 2'd3;

reg [1:0]  restore_scrub_state = RESTORE_IDLE;
reg [10:0] restore_scrub_index = 11'd0;
wire       restore_scrub_active =
    restore_scrub_state != RESTORE_IDLE;
wire       restore_clear_write =
    restore_scrub_state == RESTORE_CLEAR_WORK;
wire       restore_boot_write =
    restore_scrub_state == RESTORE_WRITE_BOOT;

assign boot_shadow_addr =
    restore_scrub_state == RESTORE_PRIME_SHADOW ? 9'd0 :
    restore_scrub_state == RESTORE_WRITE_BOOT ?
        (restore_scrub_index[8:0] == RESTORE_BOOT_LAST ?
            RESTORE_BOOT_LAST :
            restore_scrub_index[8:0] + 9'd1) :
    9'd0;

always @(posedge clk16 or posedge reset) begin
    if (reset) begin
        restore_scrub_state <= RESTORE_IDLE;
        restore_scrub_index <= 11'd0;
    end else if (ss_restore_commit) begin
        restore_scrub_state <= RESTORE_CLEAR_WORK;
        restore_scrub_index <= 11'd0;
    end else begin
        case (restore_scrub_state)
            RESTORE_CLEAR_WORK: begin
                if (restore_scrub_index == RESTORE_SCRUB_LAST) begin
                    restore_scrub_state <= RESTORE_PRIME_SHADOW;
                    restore_scrub_index <= 11'd0;
                end else begin
                    restore_scrub_index <= restore_scrub_index + 11'd1;
                end
            end
            RESTORE_PRIME_SHADOW: begin
                restore_scrub_state <= RESTORE_WRITE_BOOT;
                restore_scrub_index <= 11'd0;
            end
            RESTORE_WRITE_BOOT: begin
                if (restore_scrub_index[8:0] == RESTORE_BOOT_LAST) begin
                    restore_scrub_state <= RESTORE_IDLE;
                    restore_scrub_index <= 11'd0;
                end else begin
                    restore_scrub_index <= restore_scrub_index + 11'd1;
                end
            end
            default: begin
                restore_scrub_state <= RESTORE_IDLE;
                restore_scrub_index <= 11'd0;
            end
        endcase
    end
end

reg [1:0] sound_ready_v25_sync = 2'b00;
always @(posedge clk16 or posedge sound_reset) begin
    if (sound_reset)
        sound_ready_v25_sync <= 2'b00;
    else if (!v25_enable)
        sound_ready_v25_sync <= 2'b00;
    else
        sound_ready_v25_sync <= {sound_ready_v25_sync[0], sound_ready};
end

wire v25_reset_async = sound_reset |
                       restore_scrub_active |
                       ~v25_enable |
                       ~sound_ready_v25_sync[1] |
                       debug_hold_v25;
reg [1:0] v25_reset_pipe = 2'b11;
always @(posedge clk16 or posedge v25_reset_async) begin
    if (v25_reset_async)
        v25_reset_pipe <= 2'b11;
    else
        v25_reset_pipe <= {v25_reset_pipe[0], 1'b0};
end
wire v25_reset_n = ~v25_reset_pipe[1];

localparam [2:0]
    BGM_REPLAY_IDLE          = 3'd0,
    BGM_REPLAY_WAIT_READY    = 3'd1,
    BGM_REPLAY_WRITE_ARG     = 3'd2,
    BGM_REPLAY_WRITE_COMMAND = 3'd3,
    BGM_REPLAY_RELEASE       = 3'd4;

reg [2:0] bgm_replay_state = BGM_REPLAY_IDLE;
reg [7:0] bgm_replay_command = 8'h00;
reg [7:0] bgm_replay_argument = 8'h00;
wire bgm_replay_write_argument =
    bgm_replay_state == BGM_REPLAY_WRITE_ARG;
wire bgm_replay_write_command =
    bgm_replay_state == BGM_REPLAY_WRITE_COMMAND;
wire bgm_replay_hold_v25 =
    (bgm_replay_state == BGM_REPLAY_WRITE_ARG) ||
    (bgm_replay_state == BGM_REPLAY_WRITE_COMMAND) ||
    (bgm_replay_state == BGM_REPLAY_RELEASE);

batsugun_v25_cpu u_v25 (
    .clk          ( clk16            ),
    .reset_n      ( v25_reset_n      ),
    .clock_enable ( v25_cen && !bgm_replay_hold_v25 ),
    .port0_in     ( ~dip_b           ),
    .port1_in     ( ~region          ),
    .portt_in     ( ~dip_a           ),
    .bus_addr     ( v25_bus_addr     ),
    .bus_dout     ( v25_bus_dout     ),
    .bus_din      ( v25_bus_din      ),
    .bus_doe      ( v25_bus_doe      ),
    .bus_r_w      ( v25_bus_r_w      ),
    .bus_mreq_n   ( v25_bus_mreq_n   ),
    .bus_mstb_n   ( v25_bus_mstb_n   ),
    .bus_iostb_n  ( v25_bus_iostb_n  ),
    .halted       ( debug_halted     ),
    .fault        ( debug_fault      ),
    .debug_pc     ( debug_pc         ),
    .state_idle   ( v25_state_idle   ),
    .ss_restore_enable(1'b0),
    .ss_restore_commit(1'b0),
    .ss_data      ( 64'd0             ),
    .ss_addr      ( 32'd0             ),
    .ss_select    ( 8'd0              ),
    .ss_write     ( 1'b0              ),
    .ss_read      ( 1'b0              ),
    .ss_query     ( 1'b0              ),
    .ss_data_out  (                    ),
    .ss_ack       (                    )
);

wire v25_mem_active = ~v25_bus_mreq_n & ~v25_bus_mstb_n;
wire v25_write_active = v25_mem_active & ~v25_bus_r_w & v25_bus_doe;
reg  v25_write_active_d = 1'b0;
wire v25_write_start = v25_write_active & ~v25_write_active_d;
wire v25_shared_cs = v25_bus_addr[19];
wire v25_mailbox_ready_write =
    v25_write_start && v25_shared_cs &&
    (v25_bus_addr[14:0] == 15'h7800) &&
    (v25_bus_dout == 8'hff);

// The 68000 is restored after its current music command, but this wrapper is
// intentionally cold-restarted. Once the rebooted V25 publishes the normal
// ff mailbox acknowledgement, pause it briefly and inject the saved BGM pair
// through the same shared-RAM port used by the sound CPU.
always @(posedge clk16 or posedge reset) begin
    if (reset) begin
        bgm_replay_state <= BGM_REPLAY_IDLE;
        bgm_replay_command <= 8'h00;
        bgm_replay_argument <= 8'h00;
    end else if (ss_restore_commit) begin
        bgm_replay_state <= ss_restore_bgm_valid ?
                            BGM_REPLAY_WAIT_READY : BGM_REPLAY_IDLE;
        bgm_replay_command <= ss_restore_bgm_command;
        bgm_replay_argument <= ss_restore_bgm_argument;
    end else begin
        case (bgm_replay_state)
            BGM_REPLAY_WAIT_READY: begin
                if (v25_mailbox_ready_write)
                    bgm_replay_state <= BGM_REPLAY_WRITE_ARG;
            end
            BGM_REPLAY_WRITE_ARG:
                bgm_replay_state <= BGM_REPLAY_WRITE_COMMAND;
            BGM_REPLAY_WRITE_COMMAND:
                bgm_replay_state <= BGM_REPLAY_RELEASE;
            BGM_REPLAY_RELEASE:
                bgm_replay_state <= BGM_REPLAY_IDLE;
            default:
                bgm_replay_state <= BGM_REPLAY_IDLE;
        endcase
    end
end

assign shared_addr = restore_clear_write ?
                     RESTORE_SCRUB_FIRST + restore_scrub_index :
                     restore_boot_write ?
                     RESTORE_BOOT_FIRST + restore_scrub_index[8:0] :
                     bgm_replay_write_argument ? 15'h7801 :
                     bgm_replay_write_command ? 15'h7800 :
                     v25_bus_addr[14:0];
assign shared_dout = restore_clear_write ? 8'h00 :
                     restore_boot_write ? boot_shadow_data :
                     bgm_replay_write_argument ? bgm_replay_argument :
                     bgm_replay_write_command ? bgm_replay_command :
                     v25_bus_dout;
assign shared_we = restore_clear_write |
                   restore_boot_write |
                   bgm_replay_write_argument |
                   bgm_replay_write_command |
                   (v25_write_start & v25_shared_cs);

reg [7:0] ym_status_sync0 = 8'h00;
reg [7:0] ym_status_sync1 = 8'h00;
reg [7:0] oki_status_sync0 = 8'h00;
reg [7:0] oki_status_sync1 = 8'h00;

always @(*) begin
    v25_bus_din = 8'h00;
    if (v25_mem_active && v25_bus_r_w) begin
        if (v25_shared_cs)
            v25_bus_din = shared_din;
        else if (v25_bus_addr == YM_DATA)
            v25_bus_din = ym_status_sync1;
        else if (v25_bus_addr == OKI_DATA)
            v25_bus_din = oki_status_sync1;
    end
end

reg       ym_request_toggle = 1'b0;
reg       ym_request_a0 = 1'b0;
reg [7:0] ym_request_data = 8'h00;
reg       oki_request_toggle = 1'b0;
reg [7:0] oki_request_data = 8'h00;

always @(posedge clk16) begin
    if (!v25_reset_n) begin
        v25_write_active_d <= 1'b0;
        ym_request_toggle <= 1'b0;
        ym_request_a0 <= 1'b0;
        ym_request_data <= 8'h00;
        oki_request_toggle <= 1'b0;
        oki_request_data <= 8'h00;
        debug_ym_write <= 1'b0;
        debug_ym_a0 <= 1'b0;
        debug_ym_data <= 8'h00;
        debug_oki_write <= 1'b0;
        debug_oki_data <= 8'h00;
    end else if (ss_restore_commit) begin
        v25_write_active_d <= 1'b0;
        ym_request_toggle <= 1'b0;
        ym_request_a0 <= 1'b0;
        ym_request_data <= 8'h00;
        oki_request_toggle <= 1'b0;
        oki_request_data <= 8'h00;
        debug_ym_write <= 1'b0;
        debug_ym_a0 <= 1'b0;
        debug_ym_data <= 8'h00;
        debug_oki_write <= 1'b0;
        debug_oki_data <= 8'h00;
    end else begin
        v25_write_active_d <= v25_write_active;
        debug_ym_write <= 1'b0;
        debug_oki_write <= 1'b0;

        if (v25_write_start && !v25_shared_cs) begin
            if (v25_bus_addr == YM_ADDR || v25_bus_addr == YM_DATA) begin
                debug_ym_write <= 1'b1;
                debug_ym_a0 <= v25_bus_addr[0];
                debug_ym_data <= v25_bus_dout;
                if (!debug_block_ym) begin
                    ym_request_a0 <= v25_bus_addr[0];
                    ym_request_data <= v25_bus_dout;
                    ym_request_toggle <= ~ym_request_toggle;
                end
            end else if (v25_bus_addr == OKI_DATA) begin
                debug_oki_write <= 1'b1;
                debug_oki_data <= v25_bus_dout;
                if (!debug_block_oki) begin
                    oki_request_data <= v25_bus_dout;
                    oki_request_toggle <= ~oki_request_toggle;
                end
            end
        end
    end
end

reg [1:0] v25_enable_sound_sync = 2'b00;
always @(posedge clk_sound) begin
    if (sound_reset)
        v25_enable_sound_sync <= 2'b00;
    else
        v25_enable_sound_sync <= {v25_enable_sound_sync[0], v25_enable};
end
wire host_reset = sound_reset | ~v25_enable_sound_sync[1];

reg [5:0] ym_divider = 6'd0;
reg ym_cen = 1'b0;
reg ym_cen_p1 = 1'b0;
always @(posedge clk_sound) begin
    ym_cen <= 1'b0;
    ym_cen_p1 <= 1'b0;
    if (sound_reset) begin
        ym_divider <= 6'd0;
    end else if (!ss_hold || !sound_ready) begin
        if (ym_divider == 6'd55) begin
            ym_divider <= 6'd0;
            ym_cen <= 1'b1;
            ym_cen_p1 <= 1'b1;
        end else begin
            ym_divider <= ym_divider + 6'd1;
            if (ym_divider == 6'd27)
                ym_cen <= 1'b1;
        end
    end
end

// JT51 implements much of its state with LUT shift registers. Keep reset
// asserted for a complete 32-slot P1 rotation while the clock enables run.
reg [5:0] ym_reset_count = 6'd0;
always @(posedge clk_sound or posedge sound_reset) begin
    if (sound_reset)
        ym_reset_count <= 6'd0;
    else if (!ym_reset_count[5] && ym_cen_p1)
        ym_reset_count <= ym_reset_count + 6'd1;
end
wire ym_reset = sound_reset | ~ym_reset_count[5];
wire ym_ready = ~ym_reset;

reg [7:0] oki_accumulator = 8'd0;
reg oki_cen = 1'b0;
always @(posedge clk_sound) begin
    oki_cen <= 1'b0;
    if (sound_reset) begin
        oki_accumulator <= 8'd0;
    end else if (!ss_hold || !sound_ready) begin
        if (oki_accumulator >= 8'd181) begin
            // JT6295 expects the chip clock. 94.5 MHz * 8 / 189 = 4 MHz.
            oki_accumulator <= oki_accumulator - 8'd181;
            oki_cen <= 1'b1;
        end else begin
            oki_accumulator <= oki_accumulator + 8'd8;
        end
    end
end

// JT6295 leaves its channel acknowledge register unreset. Prime one complete
// channel rotation, then reset the rest of the decoder after acknowledge has
// reached a known value. This keeps the upstream module untouched and avoids
// relying on FPGA power-up state.
reg [1:0] oki_reset_state = 2'd0;
reg [13:0] oki_reset_timer = 14'd0;
reg oki_reset = 1'b1;
reg oki_ready = 1'b0;
always @(posedge clk_sound or posedge sound_reset) begin
    if (sound_reset) begin
        oki_reset_state <= 2'd0;
        oki_reset_timer <= 14'd0;
        oki_reset <= 1'b1;
        oki_ready <= 1'b0;
    end else begin
        case (oki_reset_state)
            2'd0: begin
                oki_reset <= 1'b0;
                if (&oki_reset_timer) begin
                    oki_reset_state <= 2'd1;
                    oki_reset_timer <= 14'd0;
                    oki_reset <= 1'b1;
                end else begin
                    oki_reset_timer <= oki_reset_timer + 14'd1;
                end
            end
            2'd1: begin
                oki_reset <= 1'b1;
                if (oki_reset_timer == 14'd15) begin
                    oki_reset_state <= 2'd2;
                    oki_reset_timer <= 14'd0;
                    oki_reset <= 1'b0;
                    oki_ready <= 1'b1;
                end else begin
                    oki_reset_timer <= oki_reset_timer + 14'd1;
                end
            end
            default: begin
                oki_reset <= 1'b0;
                oki_ready <= 1'b1;
            end
        endcase
    end
end

assign sound_ready = ym_ready & oki_ready;

reg ym_request_sync0 = 1'b0;
reg ym_request_sync1 = 1'b0;
reg ym_request_seen = 1'b0;
reg ym_request_pending = 1'b0;
reg ym_host_active = 1'b0;
reg ym_host_a0 = 1'b0;
reg [7:0] ym_host_data = 8'h00;
reg ym_cs_n = 1'b1;
reg ym_wr_n = 1'b1;

reg oki_request_sync0 = 1'b0;
reg oki_request_sync1 = 1'b0;
reg oki_request_seen = 1'b0;
reg oki_request_pending = 1'b0;
reg [7:0] oki_host_data = 8'h00;
reg oki_wr_n = 1'b1;

always @(posedge clk_sound) begin
    if (host_reset) begin
        ym_request_sync0 <= 1'b0;
        ym_request_sync1 <= 1'b0;
        ym_request_seen <= 1'b0;
        ym_request_pending <= 1'b0;
        ym_host_active <= 1'b0;
        ym_host_a0 <= 1'b0;
        ym_host_data <= 8'h00;
        ym_cs_n <= 1'b1;
        ym_wr_n <= 1'b1;
        oki_request_sync0 <= 1'b0;
        oki_request_sync1 <= 1'b0;
        oki_request_seen <= 1'b0;
        oki_request_pending <= 1'b0;
        oki_host_data <= 8'h00;
        oki_wr_n <= 1'b1;
        debug_cdc_overrun <= 1'b0;
    end else if (ss_hold) begin
        ym_cs_n <= 1'b1;
        ym_wr_n <= 1'b1;
        oki_wr_n <= 1'b1;
    end else begin
        ym_request_sync0 <= ym_request_toggle;
        ym_request_sync1 <= ym_request_sync0;
        oki_request_sync0 <= oki_request_toggle;
        oki_request_sync1 <= oki_request_sync0;
        oki_wr_n <= 1'b1;

        if (ym_request_sync1 != ym_request_seen) begin
            if (ym_request_pending || ym_host_active)
                debug_cdc_overrun <= 1'b1;
            ym_request_seen <= ym_request_sync1;
            ym_request_pending <= 1'b1;
            ym_host_a0 <= ym_request_a0;
            ym_host_data <= ym_request_data;
        end

        if (ym_request_pending && !ym_host_active) begin
            ym_request_pending <= 1'b0;
            ym_host_active <= 1'b1;
            ym_cs_n <= 1'b0;
            ym_wr_n <= 1'b0;
        end else if (ym_host_active && ym_cen_p1) begin
            ym_host_active <= 1'b0;
            ym_cs_n <= 1'b1;
            ym_wr_n <= 1'b1;
        end

        if (oki_request_sync1 != oki_request_seen) begin
            if (oki_request_pending)
                debug_cdc_overrun <= 1'b1;
            oki_request_seen <= oki_request_sync1;
            oki_request_pending <= 1'b1;
            oki_host_data <= oki_request_data;
        end

        if (oki_request_pending) begin
            oki_request_pending <= 1'b0;
            oki_wr_n <= 1'b0;
        end
    end
end

wire [7:0] ym_dout;
wire ym_sample;
wire signed [15:0] ym_left;
wire signed [15:0] ym_right;
wire signed [15:0] ym_xleft;
wire signed [15:0] ym_xright;

jt51 u_ym2151 (
    .rst        ( ym_reset      ),
    .clk        ( clk_sound     ),
    .cen        ( ym_cen        ),
    .cen_p1     ( ym_cen_p1     ),
    .cs_n       ( ym_cs_n       ),
    .wr_n       ( ym_wr_n       ),
    .a0         ( ym_host_a0    ),
    .din        ( ym_host_data  ),
    .dout       ( ym_dout       ),
    .ct1        (               ),
    .ct2        (               ),
    .irq_n      (               ),
    .sample     ( ym_sample     ),
    .left       ( ym_left       ),
    .right      ( ym_right      ),
    .xleft      ( ym_xleft      ),
    .xright     ( ym_xright     )
);

wire [7:0] oki_dout;
wire signed [13:0] oki_sound;
wire oki_sample;

jt6295 #(.INTERPOL(0)) u_oki6295 (
    .rst        ( oki_reset     ),
    .clk        ( clk_sound     ),
    .cen        ( oki_cen       ),
    .ss         ( 1'b0          ),
    .wrn        ( oki_wr_n      ),
    .din        ( oki_host_data ),
    .dout       ( oki_dout      ),
    .rom_addr   ( oki_rom_addr  ),
    .rom_data   ( oki_rom_data  ),
    .rom_ok     ( oki_rom_ok    ),
    .sound      ( oki_sound     ),
    .sample     ( oki_sample    )
);

always @(posedge clk16) begin
    if (!v25_reset_n || sound_reset) begin
        ym_status_sync0 <= 8'h00;
        ym_status_sync1 <= 8'h00;
        oki_status_sync0 <= 8'h00;
        oki_status_sync1 <= 8'h00;
    end else begin
        ym_status_sync0 <= ym_dout;
        ym_status_sync1 <= ym_status_sync0;
        oki_status_sync0 <= oki_dout;
        oki_status_sync1 <= oki_status_sync0;
    end
end

batsugun_sound_mixer u_mixer (
    .clk        ( clk_sound ),
    .reset      ( ym_reset  ),
    .sample     ( ym_sample ),
    .ym_left    ( ym_xleft  ),
    .ym_right   ( ym_xright ),
    .oki        ( oki_sound ),
    .ym_enable  ( ym_enable ),
    .oki_enable ( oki_enable),
    .oki_ready  ( oki_ready ),
    .ss_restore_write(1'b0),
    .ss_restore_data(36'd0),
    .ss_state_data(),
    .mono       ( snd_mono  )
);

assign sample = ym_sample;

wire ym_host_idle =
    !ym_request_pending &&
    !ym_host_active &&
    ym_cs_n &&
    ym_wr_n &&
    (ym_request_toggle == ym_request_sync0) &&
    (ym_request_sync0 == ym_request_sync1) &&
    (ym_request_sync1 == ym_request_seen);
wire oki_host_idle =
    !oki_request_pending &&
    oki_wr_n &&
    (oki_request_toggle == oki_request_sync0) &&
    (oki_request_sync0 == oki_request_sync1) &&
    (oki_request_sync1 == oki_request_seen);
wire sound_host_idle = ym_host_idle && oki_host_idle;
wire sound_quiet =
    sound_ready &&
    sound_host_idle &&
    !ym_cen &&
    !ym_cen_p1 &&
    !oki_cen;

reg [3:0] sound_quiet_count = 4'd0;
reg [3:0] sound_hold_count = 4'd0;
reg       sound_hold_rom_ready = 1'b0;

always @(posedge clk_sound) begin
    if (sound_reset) begin
        sound_quiet_count <= 4'd0;
        sound_hold_count <= 4'd0;
        sound_hold_rom_ready <= 1'b0;
    end else begin
        if (sound_quiet) begin
            if (!(&sound_quiet_count))
                sound_quiet_count <= sound_quiet_count + 4'd1;
        end else begin
            sound_quiet_count <= 4'd0;
        end

        if (!ss_hold) begin
            sound_hold_count <= 4'd0;
            sound_hold_rom_ready <= 1'b0;
        end else begin
            if (!(&sound_hold_count))
                sound_hold_count <= sound_hold_count + 4'd1;
            if (oki_rom_ok)
                sound_hold_rom_ready <= 1'b1;
        end
    end
end

assign sound_state_idle =
    v25_state_idle &&
    (bgm_replay_state == BGM_REPLAY_IDLE) &&
    sound_quiet_count[3] &&
    oki_rom_ok;
assign sound_state_held =
    ss_hold &&
    !restore_scrub_active &&
    sound_ready &&
    &sound_hold_count &&
    sound_hold_rom_ready &&
    sound_host_idle;

endmodule
