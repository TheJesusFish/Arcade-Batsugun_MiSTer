///////////////////////////////////////////

// MODULE batsugun_ss_jt6295_timing

module batsugun_ss_jt6295_timing (
    input               clk,
    input               cen,
    input               ss,
    output reg          cen_sr,                   // Sample rate

    output reg          cen_sr4,                  // 4x sample rate

    output reg          cen_sr4b,                 // 4x sample rate, 180 shift

    output reg          cen_sr32,
    output reg          cen_48k,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack
    // 48 kHz low pass filter

);
  genvar auto_ss_idx;

  wire device_match = (auto_ss_device_idx == auto_ss_base_device_idx);


  localparam [7:0] N48 = 8'd6, D48 = 8'd125;

  reg  [2:0] base = 0;
  reg  [5:0] cnt = 6'd0;
  wire [2:0] lim = ss ? 3'h3 : 3'h4;

  reg  [7:0] cnt48 = 0;
  wire       over48 = cnt48 >= D48;

  always @(posedge clk) begin
    begin
      if (cen) cnt48 <= over48 ? cnt48 - (D48 - N48) : cnt48 + N48;
      cen_48k <= over48 && cen;
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          cnt48   <= auto_ss_data_in[7:0];
          cen_48k <= auto_ss_data_in[17];
        end
        default: begin
        end
      endcase
    end
  end



  always @(posedge clk) begin
    begin
      cen_sr4  <= 1'd0;
      cen_sr4b <= 1'd0;
      cen_sr   <= 1'd0;
      cen_sr32 <= 1'b0;
      if (cen) begin
        base <= (base == lim) ? 3'd0 : base + 3'd1;
        if (base == 3'd0) cnt <= (cnt == 6'd32) ? 6'd0 : cnt + 6'd1;

        cen_sr32 <= !cnt[5] && base == 3'd0;
        cen_sr4  <= !cnt[5] && cnt[2:0] == 3'b000 && base == 3'd0;
        cen_sr4b <= !cnt[5] && cnt[2:0] == 3'b100 && base == 3'd0;
        cen_sr   <= {cnt, base} == 9'd0;
      end
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          cnt      <= auto_ss_data_in[13:8];
          base     <= auto_ss_data_in[16:14];
          cen_sr   <= auto_ss_data_in[18];
          cen_sr32 <= auto_ss_data_in[19];
          cen_sr4  <= auto_ss_data_in[20];
          cen_sr4b <= auto_ss_data_in[21];
        end
        default: begin
        end
      endcase
    end
  end


  always_comb begin
    auto_ss_data_out = 32'h0;
    auto_ss_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_data_out[21:0] = {cen_sr4b, cen_sr4, cen_sr32, cen_sr, cen_48k, base, cnt, cnt48};
          auto_ss_ack = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



endmodule


///////////////////////////////////////////

// MODULE batsugun_ss_jt6295_rom

module batsugun_ss_jt6295_rom (
    input rst,
    input clk,
    input cen4,
    input cen32,

    input [17:0] adpcm_addr,
    input [17:0] ctrl_addr,

    output reg [7:0] adpcm_dout,
    output reg [7:0] ctrl_dout,

    output reg          ctrl_ok,
    // ROM interface

    output reg   [17:0] rom_addr,
    input        [ 7:0] rom_data,
    input               rom_ok,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack

);
  genvar auto_ss_idx;

  wire       device_match = (auto_ss_device_idx == auto_ss_base_device_idx);


  reg  [7:0] st;
  reg  [1:0] wait2;

  always @(posedge clk) begin
    begin
      if (cen4) st <= 8'h80;
      else if (cen32) st <= {st[6:0], st[7]};
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          st <= auto_ss_data_in[25:18];
        end
        default: begin
        end
      endcase
    end
  end



  wire new_addr = rom_addr != ctrl_addr;

  always @(posedge clk) begin
    begin
      case (st)
        8'b1, 8'b10: begin
          rom_addr   <= adpcm_addr;
          adpcm_dout <= rom_data;
          ctrl_ok    <= 1'b0;
          wait2      <= 2'b0;
        end
        default: begin
          rom_addr <= ctrl_addr;
          // right after coming in rom_ok will still

          // represent the status for adpcm data

          if (wait2 == 2'b11 && !new_addr) begin
            ctrl_ok   <= rom_ok;
            ctrl_dout <= rom_data;
          end else ctrl_ok <= 1'b0;
          if (new_addr) wait2 <= 2'b0;
          else wait2 <= {wait2[0], 1'b1};
        end
      endcase
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          rom_addr <= auto_ss_data_in[17:0];
        end
        1: begin
          adpcm_dout <= auto_ss_data_in[7:0];
          ctrl_dout  <= auto_ss_data_in[15:8];
          wait2      <= auto_ss_data_in[17:16];
          ctrl_ok    <= auto_ss_data_in[18];
        end
        default: begin
        end
      endcase
    end
  end


  always_comb begin
    auto_ss_data_out = 32'h0;
    auto_ss_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_data_out[25:0] = {st, rom_addr};
          auto_ss_ack            = 1'b1;
        end
        1: begin
          auto_ss_data_out[18:0] = {ctrl_ok, wait2, ctrl_dout, adpcm_dout};
          auto_ss_ack            = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



endmodule


///////////////////////////////////////////

// MODULE batsugun_ss_jt6295_ctrl

module batsugun_ss_jt6295_ctrl (
    input               rst,
    input               clk,
    input               cen4,
    input               cen1,
    // CPU

    input               wrn,
    input        [ 7:0] din,
    // Channel address

    output reg   [17:0] start_addr,
    output reg   [17:0] stop_addr,
    // Attenuation

    output reg   [ 3:0] att,
    // ROM interface

    output       [ 9:0] rom_addr,
    input        [ 7:0] rom_data,
    input               rom_ok,
    // flow control

    output reg   [ 3:0] start,
    output reg   [ 3:0] stop,
    input        [ 3:0] busy,
    input        [ 3:0] ack,
    input               zero,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack

);
  genvar auto_ss_idx;

  wire       device_match = (auto_ss_device_idx == auto_ss_base_device_idx);


  reg        last_wrn;
  wire       negedge_wrn = !wrn && last_wrn;

  // new request

  reg  [6:0] phrase;
  reg push, pull;
  reg [3:0] ch, new_att;
  reg cmd;

  always @(posedge clk) begin
    begin
      last_wrn <= wrn;
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        4: begin
          last_wrn <= auto_ss_data_in[3];
        end
        default: begin
        end
      endcase
    end
  end



  reg stop_clr;




  // Bus interface

  always @(posedge clk) begin
    begin
      if (rst) begin
        cmd     <= 1'b0;
        stop    <= 4'd0;
        ch      <= 4'd0;
        pull    <= 1'b1;
        phrase  <= 7'd0;
        new_att <= 0;
      end else begin
        if (cen4) begin
          stop <= stop & busy;
        end
        if (push) pull <= 1'b0;
        if (negedge_wrn) begin  // new write

          if (cmd) begin  // 2nd byte

            ch      <= din[7:4];
            new_att <= din[3:0];
            cmd     <= 1'b0;
            pull    <= 1'b1;
          end else if (din[7]) begin  // channel start

            phrase <= din[6:0];
            cmd    <= 1'b1;  // wait for second byte

            stop   <= 4'd0;
          end else begin  // stop data

            stop <= din[6:3];
          end
        end
      end
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        3: begin
          phrase  <= auto_ss_data_in[6:0];
          ch      <= auto_ss_data_in[10:7];
          new_att <= auto_ss_data_in[14:11];
          stop    <= auto_ss_data_in[18:15];
        end
        4: begin
          cmd  <= auto_ss_data_in[4];
          pull <= auto_ss_data_in[5];
        end
        default: begin
        end
      endcase
    end
  end



  reg [17:0] new_start;
  reg [17:8] new_stop;
  reg [2:0] st, addr_lsb;
  reg wrom;

  assign rom_addr = {phrase, addr_lsb};

  // Request phrase address

  always @(posedge clk) begin
    begin
      if (rst) begin
        st         <= 7;
        att        <= 0;
        start_addr <= 0;
        stop_addr  <= 0;
        start      <= 0;
        push       <= 0;
        addr_lsb   <= 0;
      end else begin
        if (st != 7) begin
          wrom <= 0;
          if (!wrom && rom_ok) begin
            st       <= st + 3'd1;
            addr_lsb <= st;
            wrom     <= 1;
          end
        end
        case (st)
          7: begin
            start    <= start & ~ack;
            addr_lsb <= 0;
            if (pull) begin
              st   <= 0;
              wrom <= 1;
              push <= 1;
            end
          end
          0: ;
          1: new_start[17:16] <= rom_data[1:0];
          2: new_start[15:8] <= rom_data;
          3: new_start[7:0] <= rom_data;
          4: new_stop[17:16] <= rom_data[1:0];
          5: new_stop[15:8] <= rom_data;
          6: begin
            start      <= ch;
            start_addr <= new_start;
            stop_addr  <= {new_stop[17:8], rom_data};
            att        <= new_att;
            push       <= 0;
          end
        endcase
      end
    end
    if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          new_start <= auto_ss_data_in[17:0];
        end
        1: begin
          start_addr <= auto_ss_data_in[17:0];
        end
        2: begin
          stop_addr <= auto_ss_data_in[17:0];
          new_stop  <= auto_ss_data_in[27:18];
        end
        3: begin
          att      <= auto_ss_data_in[22:19];
          start    <= auto_ss_data_in[26:23];
          addr_lsb <= auto_ss_data_in[29:27];
        end
        4: begin
          st   <= auto_ss_data_in[2:0];
          push <= auto_ss_data_in[6];
          wrom <= auto_ss_data_in[7];
        end
        default: begin
        end
      endcase
    end
  end


  always_comb begin
    auto_ss_data_out = 32'h0;
    auto_ss_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_data_out[18-1:0] = new_start;
          auto_ss_ack              = 1'b1;
        end
        1: begin
          auto_ss_data_out[18-1:0] = start_addr;
          auto_ss_ack              = 1'b1;
        end
        2: begin
          auto_ss_data_out[27:0] = {new_stop, stop_addr};
          auto_ss_ack            = 1'b1;
        end
        3: begin
          auto_ss_data_out[29:0] = {addr_lsb, start, att, stop, new_att, ch, phrase};
          auto_ss_ack            = 1'b1;
        end
        4: begin
          auto_ss_data_out[7:0] = {wrom, push, pull, cmd, last_wrn, st};
          auto_ss_ack           = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



endmodule


///////////////////////////////////////////

// MODULE batsugun_ss_jt6295_sh_rst

module batsugun_ss_jt6295_sh_rst #(
    parameter WIDTH  = 5,
              STAGES = 32,
              RSTVAL = 1'b0
) (
    input                    rst,
    input                    clk,
    input                    clk_en  /* synthesis direct_enable */,
    input        [WIDTH-1:0] din,
    output       [WIDTH-1:0] drop,
    input                    auto_ss_rd,
    input                    auto_ss_wr,
    input        [     31:0] auto_ss_data_in,
    input        [      7:0] auto_ss_device_idx,
    input        [     15:0] auto_ss_state_idx,
    input        [      7:0] auto_ss_base_device_idx,
    output logic [     31:0] auto_ss_data_out,
    output logic             auto_ss_ack

);
  genvar auto_ss_idx;

  wire              device_match = (auto_ss_device_idx == auto_ss_base_device_idx);


  reg  [STAGES-1:0] bits                                                           [WIDTH-1:0];

  genvar i;
  integer k;
  generate
    initial
      for (k = 0; k < WIDTH; k = k + 1) begin
        bits[k] = {STAGES{RSTVAL}};
      end
  endgenerate

  generate
    for (i = 0; i < WIDTH; i = i + 1) begin : bit_shifter
      always @(posedge clk, posedge rst)
        if (rst) begin
          bits[i] <= {STAGES{RSTVAL}};
        end else if (auto_ss_wr && device_match) begin
          if (auto_ss_state_idx == (i)) begin
            bits[i] <= auto_ss_data_in[STAGES-1:0];
          end
        end else if (clk_en) begin
          bits[i] <= {bits[i][STAGES-2:0], din[i]};
        end
      assign drop[i] = bits[i][STAGES-1];
    end
  endgenerate
  always_comb begin
    auto_ss_data_out = 32'h0;
    auto_ss_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      if (auto_ss_state_idx < (WIDTH)) begin
        auto_ss_data_out[STAGES-1:0] = bits[auto_ss_state_idx];
        auto_ss_ack                  = 1'b1;
      end
    end
  end



endmodule


///////////////////////////////////////////

// MODULE batsugun_ss_jt6295_serial

module batsugun_ss_jt6295_serial (
    input               rst,
    input               clk,
    input               cen,
    input               cen4,
    // Flow

    input        [17:0] start_addr,
    input        [17:0] stop_addr,
    input        [ 3:0] att,
    input        [ 3:0] start,
    input        [ 3:0] stop,
    output reg   [ 3:0] busy,
    output reg   [ 3:0] ack,
    output              zero,
    // ADPCM data feed    

    output       [17:0] rom_addr,
    input        [ 7:0] rom_data,
    // serialized data

    output reg          pipe_en,
    output reg   [ 3:0] pipe_att,
    output reg   [ 3:0] pipe_data,
    input               auto_ss_rd,
    input               auto_ss_wr,
    input        [31:0] auto_ss_data_in,
    input        [ 7:0] auto_ss_device_idx,
    input        [15:0] auto_ss_state_idx,
    input        [ 7:0] auto_ss_base_device_idx,
    output logic [31:0] auto_ss_data_out,
    output logic        auto_ss_ack

);
  genvar auto_ss_idx;

  wire         device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  wire  [31:0] auto_ss_u_cnt_data_out;

  wire         auto_ss_u_cnt_ack;

  logic [31:0] auto_ss_local_data_out;

  logic        auto_ss_local_ack;

  assign auto_ss_data_out = auto_ss_local_data_out | auto_ss_u_cnt_data_out;

  assign auto_ss_ack      = auto_ss_local_ack | auto_ss_u_cnt_ack;


  localparam CSRW = 18 + 19 + 4 + 1;

  reg [3:0] ch;
  wire [3:0] att_in, att_out;
  wire [18:0] cnt, cnt_next, cnt_in;
  wire [17:0] ch_end, stop_in, stop_out;
  wire update;
  wire over, busy_in, busy_out, cont;
  reg up_start, up_stop;

  // current channel

  always @(posedge clk, posedge rst) begin
    if (rst) ch <= 4'b1;
    else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          ch <= auto_ss_data_in[3:0];
        end
        default: begin
        end
      endcase
    end else begin
      // if(cen4) ch <= { ch[0], ch[3:1]  };

      if (cen4) ch <= {ch[2:0], ch[3]};
    end
  end

  always @(*) begin
    case (ch)
      4'b0001: {up_start, up_stop} = {start[0], stop[0]};
      4'b0010: {up_start, up_stop} = {start[1], stop[1]};
      4'b0100: {up_start, up_stop} = {start[2], stop[2]};
      4'b1000: {up_start, up_stop} = {start[3], stop[3]};
      default: {up_start, up_stop} = 2'b00;
    endcase
  end

  reg [17:0] cnt0, cnt1, cnt2, cnt3;

  always @(posedge clk, posedge rst) begin
    if (rst) begin
      busy <= 4'd0;
    end else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          ack  <= auto_ss_data_in[7:4];
          busy <= auto_ss_data_in[11:8];
        end
        default: begin
        end
      endcase
    end else begin
      case (ch)
        4'b0001: busy[0] <= busy_in;
        4'b0010: busy[1] <= busy_in;
        4'b0100: busy[2] <= busy_in;
        4'b1000: busy[3] <= busy_in;
        default: busy <= 4'd0;
      endcase
      if (cen4) begin
        case (ch)
          4'b0001: ack <= up_start ? ch : 4'b0;
          4'b0010: ack <= up_start ? ch : 4'b0;
          4'b0100: ack <= up_start ? ch : 4'b0;
          4'b1000: ack <= up_start ? ch : 4'b0;
          default: ack <= 4'd0;
        endcase
      end

    end
  end

  assign zero     = ch[0];
  assign update   = up_start | up_stop;
  assign cont     = busy_out & ~over;
  assign cnt_next = cont ? cnt + 19'd1 : cnt;
  assign stop_in  = up_start ? stop_addr : stop_out;
  assign cnt_in   = up_start ? {start_addr, 1'b0} : cnt_next;
  assign att_in   = up_start ? att : att_out;
  assign busy_in  = update ? (up_start & ~up_stop) : cont;

  wire [CSRW-1:0] csr_in, csr_out;
  assign csr_in                             = {stop_in, cnt_in, att_in, busy_in};
  assign {stop_out, cnt, att_out, busy_out} = csr_out;
  assign rom_addr                           = cnt[18:1];
  assign over                               = rom_addr >= stop_out;

  batsugun_ss_jt6295_sh_rst #(
      .WIDTH (CSRW),
      .STAGES(4)
  ) u_cnt (
      .rst                    (rst),
      .clk                    (clk),
      .clk_en                 (cen4),
      .din                    (csr_in),
      .drop                   (csr_out),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 8'd1),
      .auto_ss_data_out       (auto_ss_u_cnt_data_out),
      .auto_ss_ack            (auto_ss_u_cnt_ack)

  );

  // Channel data is latched for a clock cycle to wait for ROM data


  always @(posedge clk, posedge rst) begin
    if (rst) begin
      pipe_data <= 4'd0;
      pipe_en   <= 1'b0;
      pipe_att  <= 4'd0;
    end else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          pipe_att  <= auto_ss_data_in[15:12];
          pipe_data <= auto_ss_data_in[19:16];
          pipe_en   <= auto_ss_data_in[20];
        end
        default: begin
        end
      endcase
    end else if (cen4) begin
      // data

      pipe_data <= !cnt[0] ? rom_data[7:4] : rom_data[3:0];
      // attenuation

      pipe_att  <= att_out;
      // busy / enable

      pipe_en   <= busy_out;
    end
  end
  always_comb begin
    auto_ss_local_data_out = 32'h0;
    auto_ss_local_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_local_data_out[20:0] = {pipe_en, pipe_data, pipe_att, busy, ack, ch};
          auto_ss_local_ack            = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



endmodule


///////////////////////////////////////////

// MODULE batsugun_ss_jt6295_adpcm

module batsugun_ss_jt6295_adpcm (
    input                    rst,
    input                    clk,
    input                    cen,
    input                    en,
    input             [ 3:0] att,
    input             [ 3:0] data,
    output reg signed [11:0] sound,
    input                    auto_ss_rd,
    input                    auto_ss_wr,
    input             [31:0] auto_ss_data_in,
    input             [ 7:0] auto_ss_device_idx,
    input             [15:0] auto_ss_state_idx,
    input             [ 7:0] auto_ss_base_device_idx,
    output logic      [31:0] auto_ss_data_out,
    output logic             auto_ss_ack

);
  genvar auto_ss_idx;

  wire         device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  wire  [31:0] auto_ss_u_enable_data_out;

  wire         auto_ss_u_enable_ack;

  wire  [31:0] auto_ss_u_att_data_out;

  wire         auto_ss_u_att_ack;

  wire  [31:0] auto_ss_u_sound_data_out;

  wire         auto_ss_u_sound_ack;

  logic [31:0] auto_ss_local_data_out;

  logic        auto_ss_local_ack;

  assign auto_ss_data_out = auto_ss_local_data_out | auto_ss_u_enable_data_out | auto_ss_u_att_data_out | auto_ss_u_sound_data_out;

  assign auto_ss_ack = auto_ss_local_ack | auto_ss_u_enable_ack | auto_ss_u_att_ack | auto_ss_u_sound_ack;



  reg [10:0] lut        [0:48];

  reg [ 5:0] idx_inc_II;
  reg [5:0] delta_idx_I, delta_idx_II, delta_idx_III, delta_idx_IV;

  reg [2:0] factor_II, factor_III;
  reg factor_IV;
  reg sign_II, sign_III, sign_IV, sign_V;
  reg [11:0] dn_II, qn_II, dn_III, qn_III, dn_IV, qn_IV, qn_V;

  always @(posedge clk, posedge rst) begin
    if (rst) begin
      idx_inc_II                           <= 6'd0;
      delta_idx_I                          <= 6'd0;
      delta_idx_II                         <= 6'd0;
      delta_idx_III                        <= 6'd0;
      delta_idx_IV                         <= 6'd0;

      factor_II                            <= 3'd0;
      factor_III                           <= 0;
      factor_IV                            <= 1'd0;
      {sign_II, sign_III, sign_IV, sign_V} <= 4'd0;
      dn_II                                <= 12'd0;
      qn_II                                <= 12'd0;
      dn_III                               <= 12'd0;
      qn_III                               <= 12'd0;
      dn_IV                                <= 12'd0;
      qn_IV                                <= 12'd0;
      qn_V                                 <= 12'd0;
    end else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          dn_II  <= auto_ss_data_in[11:0];
          dn_III <= auto_ss_data_in[23:12];
        end
        1: begin
          dn_IV <= auto_ss_data_in[11:0];
          qn_II <= auto_ss_data_in[23:12];
        end
        2: begin
          qn_III <= auto_ss_data_in[11:0];
          qn_IV  <= auto_ss_data_in[23:12];
        end
        3: begin
          qn_V <= auto_ss_data_in[11:0];
        end
        4: begin
          delta_idx_I  <= auto_ss_data_in[24:19];
          delta_idx_II <= auto_ss_data_in[30:25];
        end
        5: begin
          delta_idx_III <= auto_ss_data_in[5:0];
          delta_idx_IV  <= auto_ss_data_in[11:6];
          idx_inc_II    <= auto_ss_data_in[17:12];
          factor_II     <= auto_ss_data_in[20:18];
          factor_III    <= auto_ss_data_in[23:21];
          factor_IV     <= auto_ss_data_in[24];
          sign_II       <= auto_ss_data_in[25];
          sign_III      <= auto_ss_data_in[26];
          sign_IV       <= auto_ss_data_in[27];
          sign_V        <= auto_ss_data_in[28];
        end
        default: begin
        end
      endcase
    end else if (cen) begin
      // I

      case (data[1:0])
        2'd0: idx_inc_II <= 6'd2;
        2'd1: idx_inc_II <= 6'd4;
        2'd2: idx_inc_II <= 6'd6;
        2'd3: idx_inc_II <= 6'd8;
      endcase
      sign_II       <= data[3];
      delta_idx_II  <= en ? delta_idx_I : 6'd0;
      factor_II     <= en ? data[2:0] : 3'd0;
      qn_II         <= {1'd0, lut[delta_idx_I] >> 3};  // next value, starts at 1/8 of step size

      dn_II         <= {1'b0, lut[delta_idx_I]};  // step size

      // II

      sign_III      <= sign_II;
      delta_idx_III <= factor_II[2] ? (delta_idx_II + idx_inc_II) : (delta_idx_II - 6'd1);
      qn_III        <= factor_II[2] ? qn_II + dn_II : qn_II;
      dn_III        <= dn_II >> 1;
      factor_III    <= factor_II;
      // III

      sign_IV       <= sign_III;
      qn_IV         <= factor_III[1] ? qn_III + dn_III : qn_III;
      dn_IV         <= dn_III >> 1;
      factor_IV     <= factor_III[0];
      delta_idx_IV  <= delta_idx_III > 6'd48 ? (factor_III[2] ? 6'd48 : 6'd0) : delta_idx_III;
      // IV

      sign_V        <= sign_IV;
      qn_V          <= factor_IV ? qn_IV + dn_IV : qn_IV;
      delta_idx_I   <= delta_idx_IV;
    end
  end

  wire en_V;

  batsugun_ss_jt6295_sh_rst #(
      .WIDTH (1),
      .STAGES(4)
  ) u_enable (
      .rst                    (rst),
      .clk                    (clk),
      .clk_en                 (cen),
      .din                    (en),
      .drop                   (en_V),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 8'd1),
      .auto_ss_data_out       (auto_ss_u_enable_data_out),
      .auto_ss_ack            (auto_ss_u_enable_ack)

  );

  wire [3:0] att_V;

  batsugun_ss_jt6295_sh_rst #(
      .WIDTH (4),
      .STAGES(4)
  ) u_att (
      .rst                    (rst),
      .clk                    (clk),
      .clk_en                 (cen),
      .din                    (att),
      .drop                   (att_V),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 8'd2),
      .auto_ss_data_out       (auto_ss_u_att_data_out),
      .auto_ss_ack            (auto_ss_u_att_ack)

  );


  wire signed [11:0] snd_out;
  reg signed [11:0] snd_V, snd_VI;
  reg signed  [12:0] unlim_V;
  reg signed  [ 6:0] gain_lut                                                [0:15];
  reg signed  [ 6:0] gain_VI;  // leave the MSB for the sign

  reg                ov_V;
  wire signed [16:0] mul_VI = snd_VI * gain_VI;  // multipliers are abundant

  // in the FPGA, so I just use one.


  always @(*) begin
    unlim_V = sign_V ? {snd_out[11], snd_out} - qn_V : {snd_out[11], snd_out} + qn_V;
    ov_V  = &{snd_out[11],sign_V,~unlim_V[11]}|&{~snd_out[11],~sign_V,unlim_V[11]}; // overflow check

    if (^unlim_V[12:11]) ov_V = 1;
    snd_V = !en_V ? 12'd0 : ov_V ? {unlim_V[12], {11{~unlim_V[12]}}} : unlim_V[11:0];  // clamp

  end

  always @(posedge clk, posedge rst) begin
    if (rst) begin
      snd_VI  <= 12'd0;
      gain_VI <= 7'd0;
      sound   <= 12'd0;
    end else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        3: begin
          snd_VI <= auto_ss_data_in[23:12];
        end
        4: begin
          sound   <= auto_ss_data_in[11:0];
          gain_VI <= auto_ss_data_in[18:12];
        end
        default: begin
        end
      endcase
    end else if (cen) begin
      snd_VI  <= snd_V;
      gain_VI <= gain_lut[att_V];
      sound   <= mul_VI[16:5];
    end
  end
  always_comb begin
    auto_ss_local_data_out = 32'h0;
    auto_ss_local_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_local_data_out[23:0] = {dn_III, dn_II};
          auto_ss_local_ack            = 1'b1;
        end
        1: begin
          auto_ss_local_data_out[23:0] = {qn_II, dn_IV};
          auto_ss_local_ack            = 1'b1;
        end
        2: begin
          auto_ss_local_data_out[23:0] = {qn_IV, qn_III};
          auto_ss_local_ack            = 1'b1;
        end
        3: begin
          auto_ss_local_data_out[23:0] = {snd_VI, qn_V};
          auto_ss_local_ack            = 1'b1;
        end
        4: begin
          auto_ss_local_data_out[30:0] = {delta_idx_II, delta_idx_I, gain_VI, sound};
          auto_ss_local_ack            = 1'b1;
        end
        5: begin
          auto_ss_local_data_out[28:0] = {
            sign_V,
            sign_IV,
            sign_III,
            sign_II,
            factor_IV,
            factor_III,
            factor_II,
            idx_inc_II,
            delta_idx_IV,
            delta_idx_III
          };
          auto_ss_local_ack = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



  batsugun_ss_jt6295_sh_rst #(
      .WIDTH (12),
      .STAGES(4)
  ) u_sound (
      .rst                    (rst),
      .clk                    (clk),
      .clk_en                 (cen),
      .din                    (snd_V),
      .drop                   (snd_out),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 8'd3),
      .auto_ss_data_out       (auto_ss_u_sound_data_out),
      .auto_ss_ack            (auto_ss_u_sound_ack)

  );

  initial begin
    lut[0]  = 11'd0016;
    lut[1]  = 11'd0017;
    lut[2]  = 11'd0019;
    lut[3]  = 11'd0021;
    lut[4]  = 11'd0023;
    lut[5]  = 11'd0025;
    lut[6]  = 11'd0028;
    lut[7]  = 11'd0031;
    lut[8]  = 11'd0034;
    lut[9]  = 11'd0037;
    lut[10] = 11'd0041;
    lut[11] = 11'd0045;
    lut[12] = 11'd0050;
    lut[13] = 11'd0055;
    lut[14] = 11'd0060;
    lut[15] = 11'd0066;
    lut[16] = 11'd0073;
    lut[17] = 11'd0080;
    lut[18] = 11'd0088;
    lut[19] = 11'd0097;
    lut[20] = 11'd0107;
    lut[21] = 11'd0118;
    lut[22] = 11'd0130;
    lut[23] = 11'd0143;
    lut[24] = 11'd0157;
    lut[25] = 11'd0173;
    lut[26] = 11'd0190;
    lut[27] = 11'd0209;
    lut[28] = 11'd0230;
    lut[29] = 11'd0253;
    lut[30] = 11'd0279;
    lut[31] = 11'd0307;
    lut[32] = 11'd0337;
    lut[33] = 11'd0371;
    lut[34] = 11'd0408;
    lut[35] = 11'd0449;
    lut[36] = 11'd0494;
    lut[37] = 11'd0544;
    lut[38] = 11'd0598;
    lut[39] = 11'd0658;
    lut[40] = 11'd0724;
    lut[41] = 11'd0796;
    lut[42] = 11'd0876;
    lut[43] = 11'd0963;
    lut[44] = 11'd1060;
    lut[45] = 11'd1166;
    lut[46] = 11'd1282;
    lut[47] = 11'd1411;
    lut[48] = 11'd1552;
  end

  // Attenuation has been verified against two CPS1 boards

  //     Magic Sword         SF2               JTCPS1

  //     88617A             89626A             fbb89f0

  // set dB      delta     dB    delta    dB     delta

  // 0   1,1              0,9            -9,1

  // 1   -2,2    -3,3    -2,4    -3,3    -12,4   -3,3

  // 2   -4,9    -2,7    -5,1    -2,7    -15,2   -2,8

  // 3   -8,2    -3,3    -8,4    -3,3    -18,4   -3,2

  // 4   -11     -2,8    -11,1   -2,7    -21,2   -2,8

  // 5   -13,5   -2,5    -13,6   -2,5    -23,7   -2,5

  // 6   -16,9   -3,4    -17,2   -3,6    -27,2   -3,5

  // 7   -19,3   -2,4    -19,7   -2,5    -29,7   -2,5

  // sampled at 192kHz, using an FFT bin size of 4096 samples, power at 637 Hz

  initial begin
    gain_lut[0]  = 7'd32;
    gain_lut[1]  = 7'd22;
    gain_lut[2]  = 7'd16;
    gain_lut[3]  = 7'd11;
    gain_lut[4]  = 7'd8;
    gain_lut[5]  = 7'd6;
    gain_lut[6]  = 7'd4;
    gain_lut[7]  = 7'd3;
    gain_lut[8]  = 7'd2;
    gain_lut[9]  = 7'd0;
    gain_lut[10] = 7'd0;
    gain_lut[11] = 7'd0;
    gain_lut[12] = 7'd0;
    gain_lut[13] = 7'd0;
    gain_lut[14] = 7'd0;
    gain_lut[15] = 7'd0;
  end



endmodule


///////////////////////////////////////////

// MODULE batsugun_ss_jt6295_acc

module batsugun_ss_jt6295_acc (
    input                rst,
    input                clk,
    input                cen,
    input                cen4,
    input  signed [11:0] sound_in,
    output signed [13:0] sound_out,
    output               sample,
    input                auto_ss_rd,
    input                auto_ss_wr,
    input         [31:0] auto_ss_data_in,
    input         [ 7:0] auto_ss_device_idx,
    input         [15:0] auto_ss_state_idx,
    input         [ 7:0] auto_ss_base_device_idx,
    output logic  [31:0] auto_ss_data_out,
    output logic         auto_ss_ack

);
  genvar auto_ss_idx;

  wire device_match = (auto_ss_device_idx == auto_ss_base_device_idx);


  parameter INTERPOL = 0;

  reg signed [13:0] acc;
  reg signed [13:0] sum;

  always @(posedge clk, posedge rst) begin
    if (rst) acc <= 14'd0;
    else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          acc <= auto_ss_data_in[13:0];
        end
        default: begin
        end
      endcase
    end else if (cen4) acc <= cen ? sound_in : acc + sound_in;
  end

  always @(posedge clk, posedge rst) begin
    if (rst) sum <= 14'd0;
    else if (auto_ss_wr && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          sum <= auto_ss_data_in[27:14];
        end
        default: begin
        end
      endcase
    end else if (cen) sum <= acc;
  end
  always_comb begin
    auto_ss_data_out = 32'h0;
    auto_ss_ack      = 1'b0;
    if (auto_ss_rd && device_match) begin
      case (auto_ss_state_idx)
        0: begin
          auto_ss_data_out[27:0] = {sum, acc};
          auto_ss_ack            = 1'b1;
        end
        default: begin
        end
      endcase
    end
  end



  assign sound_out = sum;
  assign sample    = cen;

endmodule


///////////////////////////////////////////

// MODULE batsugun_ss_jt6295

module batsugun_ss_jt6295 (
    input                rst,
    input                clk,
    input                cen  /* direct_enable */,
    input                ss,                        // ss pin: selects sample rate

    // CPU interface

    input                wrn,                       // active low

    input         [ 7:0] din,
    output        [ 7:0] dout,
    // ROM interface

    output        [17:0] rom_addr,
    input         [ 7:0] rom_data,
    input                rom_ok,
    // Sound output

    output signed [13:0] sound,
    output               sample,
    input                auto_ss_rd,
    input                auto_ss_wr,
    input         [31:0] auto_ss_data_in,
    input         [ 7:0] auto_ss_device_idx,
    input         [15:0] auto_ss_state_idx,
    input         [ 7:0] auto_ss_base_device_idx,
    output logic  [31:0] auto_ss_data_out,
    output logic         auto_ss_ack
    // 48 kHz for a 1.000 MHz cen

);
  genvar auto_ss_idx;

  wire        device_match = (auto_ss_device_idx == auto_ss_base_device_idx);

  wire [31:0] auto_ss_u_timing_data_out;

  wire        auto_ss_u_timing_ack;

  wire [31:0] auto_ss_u_rom_data_out;

  wire        auto_ss_u_rom_ack;

  wire [31:0] auto_ss_u_ctrl_data_out;

  wire        auto_ss_u_ctrl_ack;

  wire [31:0] auto_ss_u_serial_data_out;

  wire        auto_ss_u_serial_ack;

  wire [31:0] auto_ss_u_adpcm_data_out;

  wire        auto_ss_u_adpcm_ack;

  wire [31:0] auto_ss_u_acc_data_out;

  wire        auto_ss_u_acc_ack;

  assign auto_ss_data_out = auto_ss_u_timing_data_out | auto_ss_u_rom_data_out | auto_ss_u_ctrl_data_out | auto_ss_u_serial_data_out | auto_ss_u_adpcm_data_out | auto_ss_u_acc_data_out;

  assign auto_ss_ack = auto_ss_u_timing_ack | auto_ss_u_rom_ack | auto_ss_u_ctrl_ack | auto_ss_u_serial_ack | auto_ss_u_adpcm_ack | auto_ss_u_acc_ack;


  parameter INTERPOL = 0;  // 0 = no interpolator

                           // 1 = 4x upsampling, LPF at 0.25*pi

                           // 2 = 4x upsampling, LPF at 0.5*pi (use if there's already)

                           //     an antialising filter after JT6295

  parameter SAMPLE = 0;  // 0 = output 48 kHz at sample pin

                         // 1 = output actual sample rate (set by SS pin and internal interpolator)


  wire cen_sr;  // sampling rate

  wire cen_sr4, cen_sr4b;  // 4x sampling rate

  wire cen_sr32,  // 32x sampling rate

  cen_48k,  // 48 kHz

  cen_eff;  // effective sound sampling rate after optional interpolator


  wire [3:0] busy, ack, start, stop;
  wire [17:0] start_addr, stop_addr, ch_addr;
  wire [9:0] ctrl_addr;
  wire [7:0] ch_data, ctrl_data;
  wire [3:0] data0, data1, data2, data3, pipe_data;
  wire [3:0] att, pipe_att;
  wire ctrl_ok, ctrl_cs, zero;
  wire               pipe_en;
  wire signed [11:0] pipe_snd;

  assign dout   = {4'hf, busy | start};
  assign sample = SAMPLE == 0 ? cen_48k : cen_eff;

  batsugun_ss_jt6295_timing u_timing (
      .clk                    (clk),
      .cen                    (cen),
      .ss                     (ss),
      .cen_sr                 (cen_sr),
      .cen_sr4                (cen_sr4),
      .cen_sr4b               (cen_sr4b),
      .cen_sr32               (cen_sr32),
      .cen_48k                (cen_48k),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 8'd1),
      .auto_ss_data_out       (auto_ss_u_timing_data_out),
      .auto_ss_ack            (auto_ss_u_timing_ack)

  );

  // ROM interface


  batsugun_ss_jt6295_rom u_rom (
      .rst                    (rst),
      .clk                    (clk),
      .cen4                   (cen_sr4),
      .cen32                  (cen_sr32),
      // Each parallel accessing device

      .adpcm_addr             (ch_addr),
      .ctrl_addr              ({8'd0, ctrl_addr}),
      // Data

      .adpcm_dout             (ch_data),
      .ctrl_dout              (ctrl_data),
      // Ok

      .ctrl_ok                (ctrl_ok),
      // ROM interface

      .rom_addr               (rom_addr),
      .rom_data               (rom_data),
      .rom_ok                 (rom_ok),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 8'd2),
      .auto_ss_data_out       (auto_ss_u_rom_data_out),
      .auto_ss_ack            (auto_ss_u_rom_ack)

  );

  // CPU interface


  batsugun_ss_jt6295_ctrl u_ctrl (
      .rst       (rst),
      .clk       (clk),
      .cen1      (cen_sr),
      .cen4      (cen_sr4),
      // CPU

      .wrn       (wrn),
      .din       (din),
      // Channel address

      .start_addr(start_addr),
      .stop_addr (stop_addr),
      // Attenuation

      .att       (att),
      // ROM interface

      .rom_addr  (ctrl_addr),
      .rom_data  (ctrl_data),
      .rom_ok    (ctrl_ok),

      .start                  (start),
      .stop                   (stop),
      .busy                   (busy),
      .ack                    (ack),
      .zero                   (zero),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 8'd3),
      .auto_ss_data_out       (auto_ss_u_ctrl_data_out),
      .auto_ss_ack            (auto_ss_u_ctrl_ack)

  );

  batsugun_ss_jt6295_serial u_serial (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen_sr),
      .cen4                   (cen_sr4),
      // Flow

      .start_addr             (start_addr),
      .stop_addr              (stop_addr),
      .att                    (att),
      .start                  (start),
      .stop                   (stop),
      .busy                   (busy),
      .ack                    (ack),
      .zero                   (zero),
      // ADPCM data feed

      .rom_addr               (ch_addr),
      .rom_data               (ch_data),
      // serialized data

      .pipe_en                (pipe_en),
      .pipe_att               (pipe_att),
      .pipe_data              (pipe_data),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 8'd4),
      .auto_ss_data_out       (auto_ss_u_serial_data_out),
      .auto_ss_ack            (auto_ss_u_serial_ack)

  );

  batsugun_ss_jt6295_adpcm u_adpcm (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen_sr4),
      // serialized data

      .en                     (pipe_en),
      .att                    (pipe_att),
      .data                   (pipe_data),
      .sound                  (pipe_snd),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 8'd6),
      .auto_ss_data_out       (auto_ss_u_adpcm_data_out),
      .auto_ss_ack            (auto_ss_u_adpcm_ack)

  );

  batsugun_ss_jt6295_acc #(
      .INTERPOL(INTERPOL)
  ) u_acc (
      .rst                    (rst),
      .clk                    (clk),
      .cen                    (cen_sr),
      .cen4                   (cen_sr4),
      // serialized data

      .sound_in               (pipe_snd),
      .sound_out              (sound),
      .sample                 (cen_eff),
      .auto_ss_rd             (auto_ss_rd),
      .auto_ss_wr             (auto_ss_wr),
      .auto_ss_data_in        (auto_ss_data_in),
      .auto_ss_device_idx     (auto_ss_device_idx),
      .auto_ss_state_idx      (auto_ss_state_idx),
      .auto_ss_base_device_idx(auto_ss_base_device_idx + 8'd10),
      .auto_ss_data_out       (auto_ss_u_acc_data_out),
      .auto_ss_ack            (auto_ss_u_acc_ack)

  );


endmodule


