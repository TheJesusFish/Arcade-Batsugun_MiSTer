// Minimal GP9001 CPU-side interface for early Batsugun bring-up.
// This models the pointer/RAM/status/register behavior the 68K POST expects.
module batsugun_gp9001_stub #(
    parameter [7:0] SS_RAM_IDX = 8'd0,
    parameter [7:0] SS_REG_IDX = 8'd0,
    parameter [7:0] SS_OBJ0_IDX = 8'd0,
    parameter [7:0] SS_OBJ1_IDX = 8'd0
) (
    input             clk,
    input             rst,

    input             start,
    input             rw,
    input      [3:0]  addr,
    input      [15:0] din,
    input      [1:0]  we_mask,
    input             status_bit,
    input      [12:0] scan_addr,
    output     [15:0] scan_dout,
    input      [12:0] obj_scan_addr,
    output     [15:0] obj_scan_dout,
    input             obj_buf_start,
    input             obj_buf_lock,
    input             obj_scan_live,
    output            obj_buf_busy,
    output            obj_forward_active,
    output            obj_forward_write,
    output            obj_forward_miss,

    output reg        busy,
    output reg        done,
    output reg [15:0] dout,
    output reg        irq_clear,

    output     [12:0] dbg_ptr,
    output reg [15:0] dbg_last_addr,
    output reg [15:0] dbg_last_din,
    output reg [15:0] dbg_last_dout,
    output reg [15:0] dbg_regs_01,
    output reg [15:0] dbg_regs_23,
    output     [15:0] scroll0,
    output     [15:0] scroll1,
    output     [15:0] scroll2,
    output     [15:0] scroll3,
    output     [15:0] scroll4,
    output     [15:0] scroll5,
    output     [15:0] scroll6,
    output     [15:0] scroll7,

    input             ss_hold,
    input             ss_restore_enable,
    input      [63:0] ss_data,
    input      [31:0] ss_addr,
    input      [7:0]  ss_select,
    input             ss_write,
    input             ss_read,
    input             ss_query,
    output     [63:0] ss_data_out,
    output            ss_ack
);

localparam ST_IDLE   = 2'd0;
localparam ST_RD_0   = 2'd1;
localparam ST_RD_1   = 2'd2;
localparam ST_RD_OUT = 2'd3;
localparam [12:0] OBJ_BASE = 13'h1800;
localparam [9:0]  OBJ_LAST = 10'h3ff;
localparam [1:0] OBJ_SYNC_IDLE  = 2'd0;
localparam [1:0] OBJ_SYNC_READ  = 2'd1;
localparam [1:0] OBJ_SYNC_WAIT  = 2'd2;
localparam [1:0] OBJ_SYNC_WRITE = 2'd3;

reg [1:0]  state = ST_IDLE;
reg [12:0] ptr = 13'h0000;
reg [7:0]  scroll_reg = 8'hff;
reg [15:0] scroll_regs [0:15];
reg [12:0] ram_addr = 13'h0000;
reg [15:0] ram_din = 16'h0000;
reg [1:0]  ram_we = 2'b00;
wire [15:0] ram_cpu_dout;
reg         obj_active_bank = 1'b0;
reg         obj_snapshot_valid = 1'b0;
reg         obj_init_active = 1'b1;
reg [9:0]   obj_init_index = 10'h000;
reg [1:0]   obj_sync_state = OBJ_SYNC_IDLE;
reg [9:0]   obj_sync_index = 10'h000;
reg [15:0]  obj_sync_data = 16'h0000;
reg         obj_sync_start_pending = 1'b0;
reg         obj_boundary_miss = 1'b0;
reg [1023:0] obj_staging_dirty_lo = {1024{1'b0}};
reg [1023:0] obj_staging_dirty_hi = {1024{1'b0}};
wire [12:0] live_scan_addr = obj_scan_live ? obj_scan_addr : scan_addr;
wire [9:0]  obj_word_index = ram_addr[9:0];
wire        obj_staging_bank = ~obj_active_bank;
wire [15:0] obj_bank0_copy_dout;
wire [15:0] obj_bank1_copy_dout;
wire [15:0] obj_bank0_scan_dout;
wire [15:0] obj_bank1_scan_dout;
wire [15:0] obj_active_copy_dout = obj_active_bank ?
                                          obj_bank1_copy_dout :
                                          obj_bank0_copy_dout;
wire [15:0] obj_snapshot_dout;
wire        ram_obj_write = (|ram_we) &&
                            (ram_addr >= OBJ_BASE) &&
                            (ram_addr <= (OBJ_BASE + {3'b000, OBJ_LAST}));
wire        obj_bank0_cpu_write = ram_obj_write &&
                                  (!obj_snapshot_valid || !obj_staging_bank);
wire        obj_bank1_cpu_write = ram_obj_write &&
                                  (!obj_snapshot_valid || obj_staging_bank);
wire        obj_init_clear_write = obj_init_active && !ram_obj_write;
wire [1:0]  obj_init_clear_mask = {
    !obj_staging_dirty_hi[obj_init_index],
    !obj_staging_dirty_lo[obj_init_index]
};
wire        obj_sync_copy_write = (obj_sync_state == OBJ_SYNC_WRITE) &&
                                   !ram_obj_write;
wire [1:0]  obj_sync_copy_mask = {
    !obj_staging_dirty_hi[obj_sync_index],
    !obj_staging_dirty_lo[obj_sync_index]
};
wire [9:0]  obj_bank0_addr = obj_bank0_cpu_write ? obj_word_index :
                              obj_init_active ? obj_init_index : obj_sync_index;
wire [9:0]  obj_bank1_addr = obj_bank1_cpu_write ? obj_word_index :
                              obj_init_active ? obj_init_index : obj_sync_index;
wire [15:0] obj_bank0_data = obj_bank0_cpu_write ? ram_din :
                              obj_init_active ? 16'h0000 : obj_sync_data;
wire [15:0] obj_bank1_data = obj_bank1_cpu_write ? ram_din :
                              obj_init_active ? 16'h0000 : obj_sync_data;
wire [1:0]  obj_bank0_we = obj_bank0_cpu_write ? ram_we :
                            obj_init_clear_write ? obj_init_clear_mask :
                            (obj_sync_copy_write && !obj_staging_bank) ?
                                obj_sync_copy_mask : 2'b00;
wire [1:0]  obj_bank1_we = obj_bank1_cpu_write ? ram_we :
                            obj_init_clear_write ? obj_init_clear_mask :
                            (obj_sync_copy_write && obj_staging_bank) ?
                                obj_sync_copy_mask : 2'b00;

wire ss_reg_selected = ss_select == SS_REG_IDX;
wire ss_reg_access = ss_reg_selected && !ss_query &&
                     (ss_read || ss_write);
wire ss_reg_restore_write = ss_reg_access && ss_write &&
                            ss_restore_enable;
reg [63:0] ss_reg_data_out = 64'd0;
reg        ss_reg_ack = 1'b0;
wire [63:0] ss_ram_data_out;
wire [63:0] ss_obj0_data_out;
wire [63:0] ss_obj1_data_out;
wire        ss_ram_ack;
wire        ss_obj0_ack;
wire        ss_obj1_ack;

assign ss_ack = ss_reg_ack || ss_ram_ack || ss_obj0_ack || ss_obj1_ack;
assign ss_data_out = ss_reg_ack ? ss_reg_data_out :
                     ss_ram_ack ? ss_ram_data_out :
                     ss_obj0_ack ? ss_obj0_data_out :
                     ss_obj1_ack ? ss_obj1_data_out :
                     64'd0;

wire [3:0] op = {addr[3:2], 2'b00};
wire       op_ram_data = (op == 4'h4);
wire       op_select   = (op == 4'h8);
wire       op_status   = (op == 4'hc);

integer i;

assign dbg_ptr = ptr;
assign scroll0 = scroll_regs[0];
assign scroll1 = scroll_regs[1];
assign scroll2 = scroll_regs[2];
assign scroll3 = scroll_regs[3];
assign scroll4 = scroll_regs[4];
assign scroll5 = scroll_regs[5];
assign scroll6 = scroll_regs[6];
assign scroll7 = scroll_regs[7];
assign obj_snapshot_dout = obj_active_bank ? obj_bank1_scan_dout :
                                                obj_bank0_scan_dout;
assign obj_scan_dout = obj_scan_live ? scan_dout : obj_snapshot_dout;
assign obj_buf_busy = obj_init_active || obj_sync_start_pending ||
                      (obj_sync_state != OBJ_SYNC_IDLE);
assign obj_forward_active = obj_snapshot_valid;
assign obj_forward_write = obj_snapshot_valid && ram_obj_write;
assign obj_forward_miss = obj_boundary_miss;

function [15:0] merge_word;
    input [15:0] old_word;
    input [15:0] new_word;
    input [1:0]  mask;
    begin
        merge_word = {
            mask[1] ? new_word[15:8] : old_word[15:8],
            mask[0] ? new_word[7:0]  : old_word[7:0]
        };
    end
endfunction

wire [15:0] ptr_merged = merge_word({3'b000, ptr}, din, we_mask);
wire [15:0] scroll_select_merged = merge_word({8'h00, scroll_reg}, din, we_mask);
wire [15:0] scroll_data_merged = merge_word(scroll_regs[scroll_reg[3:0]], din, we_mask);

// Register chunk: pointer, scroll selector, sixteen scroll registers, then
// the object snapshot bank/valid flags. Transient bus and copy states are
// standardized to idle at the save-state quiesce point.
always @(posedge clk) begin
    ss_reg_ack <= 1'b0;

    if (ss_reg_selected && ss_query) begin
        ss_reg_data_out <= {SS_REG_IDX, 22'd0, 2'd1, 32'd19};
        ss_reg_ack <= 1'b1;
    end else if (ss_reg_access) begin
        if (ss_write) begin
            ss_reg_ack <= 1'b1;
        end else if (ss_read) begin
            if (ss_addr == 32'd0)
                ss_reg_data_out <= {48'd0, 3'd0, ptr};
            else if (ss_addr == 32'd1)
                ss_reg_data_out <= {56'd0, scroll_reg};
            else if (ss_addr < 32'd18)
                ss_reg_data_out <= {
                    48'd0,
                    scroll_regs[ss_addr[3:0] - 4'd2]
                };
            else
                ss_reg_data_out <= {
                    62'd0,
                    obj_snapshot_valid,
                    obj_active_bank
                };
            ss_reg_ack <= 1'b1;
        end
    end
end

always @(posedge clk) begin
    done <= 1'b0;
    irq_clear <= 1'b0;
    ram_we <= 2'b00;

    if (rst) begin
        busy <= 1'b0;
        state <= ST_IDLE;
        ptr <= 13'h0000;
        scroll_reg <= 8'hff;
        dout <= 16'hffff;
        dbg_last_addr <= 16'h0000;
        dbg_last_din <= 16'h0000;
        dbg_last_dout <= 16'h0000;
        dbg_regs_01 <= 16'h0000;
        dbg_regs_23 <= 16'h0000;
        for (i = 0; i < 16; i = i + 1) begin
            scroll_regs[i] <= 16'h0000;
        end
    end else if (ss_reg_restore_write && (ss_addr < 32'd18)) begin
        busy <= 1'b0;
        state <= ST_IDLE;
        if (ss_addr == 32'd0)
            ptr <= ss_data[12:0];
        else if (ss_addr == 32'd1)
            scroll_reg <= ss_data[7:0];
        else
            scroll_regs[ss_addr[3:0] - 4'd2] <= ss_data[15:0];
    end else if (!ss_hold) begin
        case (state)
            ST_IDLE: begin
                if (start && !busy) begin
                    busy <= 1'b1;
                    dbg_last_addr <= {rw, 11'h000, addr};
                    dbg_last_din <= din;

                    if (rw) begin
                        if (op_ram_data) begin
                            ram_addr <= ptr;
                            ptr <= ptr + 13'd1;
                            state <= ST_RD_0;
                        end else if (op_status) begin
                            dout <= {15'h0000, status_bit};
                            dbg_last_dout <= {15'h0000, status_bit};
                            done <= 1'b1;
                            busy <= 1'b0;
                        end else begin
                            dout <= 16'hffff;
                            dbg_last_dout <= 16'hffff;
                            done <= 1'b1;
                            busy <= 1'b0;
                        end
                    end else begin
                        if (op == 4'h0) begin
                            ptr <= ptr_merged[12:0];
                        end else if (op_ram_data) begin
                            ram_addr <= ptr;
                            ram_din <= din;
                            ram_we <= we_mask;
                            ptr <= ptr + 13'd1;
                        end else if (op_select) begin
                            scroll_reg <= scroll_select_merged[7:0] & 8'h8f;
                        end else if (op_status) begin
                            scroll_regs[scroll_reg[3:0]] <= scroll_data_merged;
                            if (scroll_reg[3:0] == 4'h0) dbg_regs_01[15:8] <= scroll_data_merged[7:0];
                            if (scroll_reg[3:0] == 4'h1) dbg_regs_01[7:0] <= scroll_data_merged[7:0];
                            if (scroll_reg[3:0] == 4'h2) dbg_regs_23[15:8] <= scroll_data_merged[7:0];
                            if (scroll_reg[3:0] == 4'h3) dbg_regs_23[7:0] <= scroll_data_merged[7:0];
                            if ((scroll_reg == 8'h0e) || (scroll_reg == 8'h0f) || (scroll_reg == 8'h8f)) begin
                                irq_clear <= 1'b1;
                            end
                        end

                        dout <= 16'hffff;
                        dbg_last_dout <= 16'hffff;
                        done <= 1'b1;
                        busy <= 1'b0;
                    end
                end
            end

            ST_RD_0: begin
                state <= ST_RD_1;
            end

            ST_RD_1: begin
                state <= ST_RD_OUT;
            end

            default: begin
                dout <= ram_cpu_dout;
                dbg_last_dout <= ram_cpu_dout;
                done <= 1'b1;
                busy <= 1'b0;
                state <= ST_IDLE;
            end
        endcase
    end
end

// GP9001 object RAM is double buffered at the rising vblank edge. CPU writes
// update the staging bank while the renderer scans a frozen active bank. The
// old active bank is refreshed in the background, with a dirty bit protecting
// every post-swap CPU write from the refresh copy.
always @(posedge clk) begin
    if (rst) begin
        obj_active_bank <= 1'b0;
        obj_snapshot_valid <= 1'b0;
        obj_init_active <= 1'b1;
        obj_init_index <= 10'h000;
        obj_sync_state <= OBJ_SYNC_IDLE;
        obj_sync_index <= 10'h000;
        obj_sync_data <= 16'h0000;
        obj_sync_start_pending <= 1'b0;
        obj_boundary_miss <= 1'b0;
        obj_staging_dirty_lo <= {1024{1'b0}};
        obj_staging_dirty_hi <= {1024{1'b0}};
    end else if (ss_reg_restore_write && (ss_addr == 32'd18)) begin
        obj_active_bank <= ss_data[0];
        obj_snapshot_valid <= ss_data[1];
        obj_init_active <= 1'b0;
        obj_init_index <= 10'h000;
        obj_sync_state <= OBJ_SYNC_IDLE;
        obj_sync_index <= 10'h000;
        obj_sync_data <= 16'h0000;
        obj_sync_start_pending <= 1'b0;
        obj_boundary_miss <= 1'b0;
        obj_staging_dirty_lo <= {1024{1'b0}};
        obj_staging_dirty_hi <= {1024{1'b0}};
    end else if (!ss_hold) begin
        obj_boundary_miss <= 1'b0;

        if (obj_buf_start) begin
            if (!obj_init_active && !obj_sync_start_pending &&
                (obj_sync_state == OBJ_SYNC_IDLE)) begin
                obj_active_bank <= ~obj_active_bank;
                obj_sync_start_pending <= 1'b1;
                obj_sync_index <= 10'h000;
                obj_staging_dirty_lo <= {1024{1'b0}};
                obj_staging_dirty_hi <= {1024{1'b0}};
                obj_snapshot_valid <= 1'b1;
            end else begin
                obj_boundary_miss <= 1'b1;
            end
        end else begin
            if (ram_obj_write) begin
                if (ram_we[0]) obj_staging_dirty_lo[obj_word_index] <= 1'b1;
                if (ram_we[1]) obj_staging_dirty_hi[obj_word_index] <= 1'b1;
            end

            if (obj_init_active) begin
                if (!ram_obj_write) begin
                    if (obj_init_index == OBJ_LAST) begin
                        obj_init_active <= 1'b0;
                        obj_staging_dirty_lo <= {1024{1'b0}};
                        obj_staging_dirty_hi <= {1024{1'b0}};
                    end else begin
                        obj_init_index <= obj_init_index + 10'd1;
                    end
                end
            end else if (obj_sync_start_pending) begin
                obj_sync_start_pending <= 1'b0;
                obj_sync_state <= OBJ_SYNC_READ;
                obj_sync_index <= 10'h000;
            end else begin
                case (obj_sync_state)
                    OBJ_SYNC_READ: begin
                        obj_sync_state <= OBJ_SYNC_WAIT;
                    end

                    OBJ_SYNC_WAIT: begin
                        obj_sync_data <= obj_active_copy_dout;
                        obj_sync_state <= OBJ_SYNC_WRITE;
                    end

                    OBJ_SYNC_WRITE: begin
                        if (!ram_obj_write) begin
                            if (obj_sync_index == OBJ_LAST) begin
                                obj_sync_state <= OBJ_SYNC_IDLE;
                            end else begin
                                obj_sync_index <= obj_sync_index + 10'd1;
                                obj_sync_state <= OBJ_SYNC_READ;
                            end
                        end
                    end

                    default: begin
                    end
                endcase
            end
        end
    end
end

wire [12:0] ss_gp_ram_addr;
wire [15:0] ss_gp_ram_data;
wire [ 1:0] ss_gp_ram_we;

batsugun_ss_ram_port #(
    .WIDTH        ( 16         ),
    .ADDR_WIDTH   ( 13         ),
    .WE_WIDTH     ( 2          ),
    .SS_IDX       ( SS_RAM_IDX ),
    .STREAM_WIDTH ( 2'd1       )
) u_gp_ram_ss (
    .clk            ( clk               ),
    .restore_enable ( ss_restore_enable ),
    .normal_we      ( 2'b00             ),
    .normal_addr    ( live_scan_addr    ),
    .normal_data    ( 16'd0             ),
    .ram_we         ( ss_gp_ram_we      ),
    .ram_addr       ( ss_gp_ram_addr    ),
    .ram_data       ( ss_gp_ram_data    ),
    .ram_q          ( scan_dout         ),
    .ss_data        ( ss_data           ),
    .ss_addr        ( ss_addr           ),
    .ss_select      ( ss_select         ),
    .ss_write       ( ss_write          ),
    .ss_read        ( ss_read           ),
    .ss_query       ( ss_query          ),
    .ss_data_out    ( ss_ram_data_out   ),
    .ss_ack         ( ss_ram_ack        )
);

jtframe_dual_ram16 #(.AW(13)) u_gp_ram (
    .clk0       ( clk       ),
    .data0      ( ram_din   ),
    .addr0      ( ram_addr  ),
    .we0        ( ram_we    ),
    .q0         ( ram_cpu_dout ),
    .clk1       ( clk       ),
    .data1      ( ss_gp_ram_data ),
    .addr1      ( ss_gp_ram_addr ),
    .we1        ( ss_gp_ram_we ),
    .q1         ( scan_dout )
);

wire [9:0]  ss_obj0_addr;
wire [15:0] ss_obj0_data;
wire [1:0]  ss_obj0_we;

batsugun_ss_ram_port #(
    .WIDTH        ( 16          ),
    .ADDR_WIDTH   ( 10          ),
    .WE_WIDTH     ( 2           ),
    .SS_IDX       ( SS_OBJ0_IDX ),
    .STREAM_WIDTH ( 2'd1        )
) u_obj0_ss (
    .clk            ( clk               ),
    .restore_enable ( ss_restore_enable ),
    .normal_we      ( 2'b00             ),
    .normal_addr    ( obj_scan_addr[9:0] ),
    .normal_data    ( 16'd0             ),
    .ram_we         ( ss_obj0_we        ),
    .ram_addr       ( ss_obj0_addr      ),
    .ram_data       ( ss_obj0_data      ),
    .ram_q          ( obj_bank0_scan_dout ),
    .ss_data        ( ss_data           ),
    .ss_addr        ( ss_addr           ),
    .ss_select      ( ss_select         ),
    .ss_write       ( ss_write          ),
    .ss_read        ( ss_read           ),
    .ss_query       ( ss_query          ),
    .ss_data_out    ( ss_obj0_data_out  ),
    .ss_ack         ( ss_obj0_ack       )
);

jtframe_dual_ram16 #(.AW(10)) u_gp_obj_ram0 (
    .clk0       ( clk       ),
    .data0      ( obj_bank0_data ),
    .addr0      ( obj_bank0_addr ),
    .we0        ( obj_bank0_we ),
    .q0         ( obj_bank0_copy_dout ),
    .clk1       ( clk       ),
    .data1      ( ss_obj0_data ),
    .addr1      ( ss_obj0_addr ),
    .we1        ( ss_obj0_we ),
    .q1         ( obj_bank0_scan_dout )
);

wire [9:0]  ss_obj1_addr;
wire [15:0] ss_obj1_data;
wire [1:0]  ss_obj1_we;

batsugun_ss_ram_port #(
    .WIDTH        ( 16          ),
    .ADDR_WIDTH   ( 10          ),
    .WE_WIDTH     ( 2           ),
    .SS_IDX       ( SS_OBJ1_IDX ),
    .STREAM_WIDTH ( 2'd1        )
) u_obj1_ss (
    .clk            ( clk               ),
    .restore_enable ( ss_restore_enable ),
    .normal_we      ( 2'b00             ),
    .normal_addr    ( obj_scan_addr[9:0] ),
    .normal_data    ( 16'd0             ),
    .ram_we         ( ss_obj1_we        ),
    .ram_addr       ( ss_obj1_addr      ),
    .ram_data       ( ss_obj1_data      ),
    .ram_q          ( obj_bank1_scan_dout ),
    .ss_data        ( ss_data           ),
    .ss_addr        ( ss_addr           ),
    .ss_select      ( ss_select         ),
    .ss_write       ( ss_write          ),
    .ss_read        ( ss_read           ),
    .ss_query       ( ss_query          ),
    .ss_data_out    ( ss_obj1_data_out  ),
    .ss_ack         ( ss_obj1_ack       )
);

jtframe_dual_ram16 #(.AW(10)) u_gp_obj_ram1 (
    .clk0       ( clk       ),
    .data0      ( obj_bank1_data ),
    .addr0      ( obj_bank1_addr ),
    .we0        ( obj_bank1_we ),
    .q0         ( obj_bank1_copy_dout ),
    .clk1       ( clk       ),
    .data1      ( ss_obj1_data ),
    .addr1      ( ss_obj1_addr ),
    .we1        ( ss_obj1_we ),
    .q1         ( obj_bank1_scan_dout )
);

endmodule
