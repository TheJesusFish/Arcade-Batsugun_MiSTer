// Batsugun-local raw state-bus adaptor for generated automatic state ports.
module batsugun_auto_state_adaptor #(
    parameter [7:0] SS_IDX = 8'd0,
    parameter integer COUNT_WIDTH = 9,
    parameter integer REVERSE = 0
) (
    input              clk,
    input              reset,
    input              restore_enable,

    input      [63:0]  ss_data,
    input      [31:0]  ss_addr,
    input      [7:0]   ss_select,
    input              ss_write,
    input              ss_read,
    input              ss_query,
    output reg [63:0]  ss_data_out,
    output reg         ss_ack,
    output reg         ready,

    output reg         auto_rd,
    output reg         auto_wr,
    output reg [31:0]  auto_data_in,
    output reg [7:0]   auto_device_idx,
    output reg [15:0]  auto_state_idx,
    input      [31:0]  auto_data_out,
    input              auto_ack
);

localparam integer MAP_DEPTH = 1 << COUNT_WIDTH;

localparam [3:0] ST_SCAN_WAIT    = 4'd0;
localparam [3:0] ST_SCAN_CHECK   = 4'd1;
localparam [3:0] ST_READY        = 4'd2;
localparam [3:0] ST_PREP_WRITE   = 4'd3;
localparam [3:0] ST_ISSUE_WRITE  = 4'd4;
localparam [3:0] ST_FINISH_WRITE = 4'd5;
localparam [3:0] ST_PREP_READ    = 4'd6;
localparam [3:0] ST_ISSUE_READ   = 4'd7;
localparam [3:0] ST_FINISH_READ  = 4'd8;
localparam [3:0] ST_WAIT_IDLE    = 4'd9;

reg [3:0] state = ST_SCAN_WAIT;
reg [COUNT_WIDTH-1:0] map_count = {COUNT_WIDTH{1'b0}};
reg [COUNT_WIDTH-1:0] access_addr = {COUNT_WIDTH{1'b0}};
reg [23:0] idx_map [0:MAP_DEPTH-1];

wire selected = ss_select == SS_IDX;
wire request_active = ss_query || ss_read || ss_write;
wire [31:0] map_count_ext =
    {{(32-COUNT_WIDTH){1'b0}}, map_count};

always @(posedge clk) begin
    ss_ack <= 1'b0;

    if (reset) begin
        state <= ST_SCAN_WAIT;
        map_count <= {COUNT_WIDTH{1'b0}};
        access_addr <= {COUNT_WIDTH{1'b0}};
        ready <= 1'b0;
        auto_rd <= 1'b1;
        auto_wr <= 1'b0;
        auto_data_in <= 32'd0;
        auto_device_idx <= 8'd0;
        auto_state_idx <= 16'd0;
        ss_data_out <= 64'd0;
    end else begin
        case (state)
            ST_SCAN_WAIT: begin
                auto_rd <= 1'b1;
                auto_wr <= 1'b0;
                state <= ST_SCAN_CHECK;
            end

            ST_SCAN_CHECK: begin
                if (auto_ack) begin
                    idx_map[map_count] <= {
                        auto_device_idx,
                        auto_state_idx
                    };
                    map_count <= map_count + {{(COUNT_WIDTH-1){1'b0}}, 1'b1};
                    auto_state_idx <= auto_state_idx + 16'd1;
                    state <= ST_SCAN_WAIT;
                end else begin
                    auto_state_idx <= 16'd0;
                    if (&auto_device_idx) begin
                        auto_rd <= 1'b0;
                        ready <= 1'b1;
                        state <= ST_READY;
                    end else begin
                        auto_device_idx <= auto_device_idx + 8'd1;
                        state <= ST_SCAN_WAIT;
                    end
                end
            end

            ST_READY: begin
                auto_rd <= 1'b0;
                auto_wr <= 1'b0;

                if (selected && ss_query) begin
                    ss_data_out <= {
                        SS_IDX,
                        22'd0,
                        2'd2,
                        map_count_ext
                    };
                    ss_ack <= 1'b1;
                    state <= ST_WAIT_IDLE;
                end else if (selected && (ss_read || ss_write)) begin
                    if (ss_addr < map_count_ext) begin
                        if (ss_write && !restore_enable) begin
                            ss_data_out <= 64'd0;
                            ss_ack <= 1'b1;
                            state <= ST_WAIT_IDLE;
                        end else begin
                            if (REVERSE != 0)
                                access_addr <= map_count -
                                               {{(COUNT_WIDTH-1){1'b0}}, 1'b1} -
                                               ss_addr[COUNT_WIDTH-1:0];
                            else
                                access_addr <= ss_addr[COUNT_WIDTH-1:0];
                            auto_data_in <= ss_data[31:0];
                            state <= ss_write ? ST_PREP_WRITE : ST_PREP_READ;
                        end
                    end else begin
                        ss_data_out <= 64'd0;
                        ss_ack <= 1'b1;
                        state <= ST_WAIT_IDLE;
                    end
                end
            end

            ST_PREP_WRITE: begin
                {
                    auto_device_idx,
                    auto_state_idx
                } <= idx_map[access_addr];
                state <= ST_ISSUE_WRITE;
            end

            ST_ISSUE_WRITE: begin
                auto_wr <= 1'b1;
                state <= ST_FINISH_WRITE;
            end

            ST_FINISH_WRITE: begin
                auto_wr <= 1'b0;
                ss_ack <= 1'b1;
                state <= ST_WAIT_IDLE;
            end

            ST_PREP_READ: begin
                {
                    auto_device_idx,
                    auto_state_idx
                } <= idx_map[access_addr];
                state <= ST_ISSUE_READ;
            end

            ST_ISSUE_READ: begin
                auto_rd <= 1'b1;
                state <= ST_FINISH_READ;
            end

            ST_FINISH_READ: begin
                if (auto_ack) begin
                    auto_rd <= 1'b0;
                    ss_data_out <= {32'd0, auto_data_out};
                    ss_ack <= 1'b1;
                    state <= ST_WAIT_IDLE;
                end
            end

            ST_WAIT_IDLE: begin
                auto_rd <= 1'b0;
                auto_wr <= 1'b0;
                if (!request_active)
                    state <= ST_READY;
            end

            default: begin
                state <= ST_SCAN_WAIT;
                ready <= 1'b0;
                auto_rd <= 1'b1;
                auto_wr <= 1'b0;
                auto_device_idx <= 8'd0;
                auto_state_idx <= 16'd0;
                map_count <= {COUNT_WIDTH{1'b0}};
            end
        endcase
    end
end

endmodule
