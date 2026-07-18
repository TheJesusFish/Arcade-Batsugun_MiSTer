-- SPDX-License-Identifier: BSD-3-Clause
--
-- First chip-level wrapper for the experimental NEC V25-compatible core.
--
-- This ties the CPU core's abstract memory/I/O ports to the external bus timing
-- adapter so fetch, data, and I/O cycles share one byte-wide timed bus shell,
-- and carries first-pass peripheral event pins such as serial receive.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.v25_pkg.all;

entity v25_chip is
  generic (
    RESET_PS   : word_t := x"FFFF";
    RESET_IP   : word_t := x"0000";
    RESET_BANK : natural range 0 to 7 := 7;
    ENABLE_TIMING_THROTTLE : boolean := false;
    FIXED_INSTRUCTION_BUDGET : natural range 0 to 31 := 0;
    ENABLE_PREFETCH_QUEUE : boolean := false;
    ENABLE_SERIAL_BAUD : boolean := false;
    ENABLE_TIMER_PRESCALER : boolean := false;
    MIN_WAIT_STATES : natural := 0
  );
  port (
    clk     : in std_logic;
    reset_n : in std_logic;
    clock_enable : in std_logic := '1';

    wait_states : in unsigned(3 downto 0) := (others => '0');
    ready       : in std_logic := '1';
    hold_req    : in std_logic := '0';
    hold_ack    : out std_logic;

    irq_request : in std_logic := '0';
    irq_vector  : in byte_t := x"00";
    intak_n_out : out std_logic := '1';
    int_ack_active_out : out std_logic := '0';
    int_ack_second_out : out std_logic := '0';
    nmi_in      : in std_logic := '0';
    intp0_in    : in std_logic := '0';
    intp1_in    : in std_logic := '0';
    intp2_in    : in std_logic := '0';

    serial0_rx_valid_in : in std_logic := '0';
    serial0_rx_data_in  : in byte_t := x"00";
    serial0_rx_frame_error_in : in std_logic := '0';
    serial0_rx_parity_error_in : in std_logic := '0';
    serial0_rxd_in      : in std_logic := '1';
    serial0_rx_tick_in  : in std_logic := '0';
    serial0_cts_in      : in std_logic := '0';
    serial1_rx_valid_in : in std_logic := '0';
    serial1_rx_data_in  : in byte_t := x"00";
    serial1_rx_frame_error_in : in std_logic := '0';
    serial1_rx_parity_error_in : in std_logic := '0';
    serial1_rxd_in      : in std_logic := '1';
    serial1_rx_tick_in  : in std_logic := '0';
    serial1_cts_in      : in std_logic := '0';

    port0_in  : in byte_t := x"00";
    port1_in  : in byte_t := x"00";
    port2_in  : in byte_t := x"00";
    portt_in  : in byte_t := x"00";
    port0_out : out byte_t := x"00";
    port1_out : out byte_t := x"00";
    port2_out : out byte_t := x"00";
    portt_out : out byte_t := x"00";
    port0_mode_control : out byte_t := x"00";
    port1_mode_control : out byte_t := x"00";
    port2_mode_control : out byte_t := x"00";
    clkout_out : out std_logic := '0';
    timer0_tick_in : in std_logic := '0';
    timer0_md_tick_in : in std_logic := '0';
    timer1_tick_in : in std_logic := '0';
    tout_out       : out std_logic := '1';

    dma0_control : out byte_t := x"00";
    dma0_mode    : out byte_t := x"00";
    dma0_start   : out std_logic := '0';
    dma0_hold    : out std_logic := '0';
    dmarq0_in    : in std_logic := '0';
    dmaaak0_n_out : out std_logic := '1';
    tc0_n_out    : out std_logic := '1';
    dma1_control : out byte_t := x"00";
    dma1_mode    : out byte_t := x"00";
    dma1_start   : out std_logic := '0';
    dma1_hold    : out std_logic := '0';
    dmarq1_in    : in std_logic := '0';
    dmaaak1_n_out : out std_logic := '1';
    tc1_n_out    : out std_logic := '1';

    serial0_rxb  : out byte_t := x"00";
    serial0_txb  : out byte_t := x"00";
    serial0_mode : out byte_t := x"00";
    serial0_ctrl : out byte_t := x"00";
    serial0_brg  : out byte_t := x"00";
    serial0_enable : out byte_t := x"00";
    serial1_rxb  : out byte_t := x"00";
    serial1_txb  : out byte_t := x"00";
    serial1_mode : out byte_t := x"00";
    serial1_ctrl : out byte_t := x"00";
    serial1_brg  : out byte_t := x"00";
    serial1_enable : out byte_t := x"00";
    serial0_tx_tick_in : in std_logic := '0';
    serial0_txd_out : out std_logic := '1';
    serial0_sck_out : out std_logic := '0';
    serial1_tx_tick_in : in std_logic := '0';
    serial1_txd_out : out std_logic := '1';

    standby_control : out byte_t := x"00";
    ram_control     : out byte_t := x"00";
    flag_control    : out byte_t := x"00";
    protect_control : out byte_t := x"00";
    timebase_control : out byte_t := x"00";

    interrupt_mode : out byte_t := x"00";
    interrupt_pending : out byte_t := x"00";
    ext_irq0_control : out byte_t := x"00";
    ext_irq1_control : out byte_t := x"00";
    ext_irq2_control : out byte_t := x"00";
    timer0_irq_control : out byte_t := x"00";
    timer1_irq_control : out byte_t := x"00";
    timer2_irq_control : out byte_t := x"00";
    timer0_control : out byte_t := x"00";
    timer1_control : out byte_t := x"00";
    dma0_irq_control : out byte_t := x"00";
    dma1_irq_control : out byte_t := x"00";
    serial0_error_irq_control : out byte_t := x"00";
    serial0_rx_irq_control : out byte_t := x"00";
    serial0_tx_irq_control : out byte_t := x"00";
    serial1_error_irq_control : out byte_t := x"00";
    serial1_rx_irq_control : out byte_t := x"00";
    serial1_tx_irq_control : out byte_t := x"00";

    bus_addr    : out addr20_t;
    bus_dout    : out byte_t;
    bus_din     : in  byte_t;
    bus_doe     : out std_logic;
    bus_r_w     : out std_logic;
    bus_mreq_n  : out std_logic;
    bus_mstb_n  : out std_logic;
    bus_iostb_n : out std_logic;
    refrq_n_out : out std_logic := '1';
    refresh_active_out : out std_logic := '0';
    refresh_addr_out : out addr20_t := (others => '0');

    halted : out std_logic;
    fault  : out std_logic;

    debug_pc  : out addr20_t;
    debug_psw : out word_t;
    debug_ax  : out word_t;
    debug_cx  : out word_t;
    debug_dx  : out word_t;
    debug_bx  : out word_t;
    debug_sp  : out word_t
  );
end entity;

architecture rtl of v25_chip is
  signal core_mem_valid : std_logic;
  signal core_mem_write : std_logic;
  signal core_mem_addr  : addr20_t;
  signal core_mem_wdata : byte_t;
  signal core_mem_rdata : byte_t;
  signal core_mem_ready : std_logic;

  signal core_io_valid : std_logic;
  signal core_io_write : std_logic;
  signal core_io_addr  : word_t;
  signal core_io_wdata : byte_t;
  signal core_io_rdata : byte_t;
  signal core_io_ready : std_logic;
  signal core_int_ack_valid : std_logic;
  signal core_int_ack_second : std_logic;
  signal core_int_ack_ready : std_logic;
  signal core_int_ack_vector : byte_t;
  signal core_int_ack_vector_valid : std_logic;
  signal irq_request_eff : std_logic;
  signal intak_n : std_logic;
  signal core_serial0_rx_valid : std_logic;
  signal core_serial0_rx_data : byte_t;
  signal core_serial0_rx_frame_error : std_logic;
  signal core_serial0_rx_parity_error : std_logic;
  signal core_serial1_rx_valid : std_logic;
  signal core_serial1_rx_data : byte_t;
  signal core_serial1_rx_frame_error : std_logic;
  signal core_serial1_rx_parity_error : std_logic;
  signal core_wait_states : unsigned(3 downto 0);
  signal core_ready_extend : std_logic;
  signal core_tout : std_logic;
  signal core_port0_mode_control : byte_t := x"00";
  signal core_port1_mode_control : byte_t := x"00";
  signal core_port2_mode_control : byte_t := x"00";
  signal core_timer0_control : byte_t := x"00";
  signal core_timer1_control : byte_t := x"00";
  signal timer0_tick_eff : std_logic;
  signal timer0_md_tick_eff : std_logic;
  signal timer1_tick_eff : std_logic;
  signal timer0_prescale_tick : std_logic := '0';
  signal timer0_md_prescale_tick : std_logic := '0';
  signal timer1_prescale_tick : std_logic := '0';
  signal timer0_prescale_count : natural range 0 to 127 := 0;
  signal timer0_md_prescale_count : natural range 0 to 127 := 0;
  signal timer1_prescale_count : natural range 0 to 127 := 0;
  signal core_dmaaak0_n : std_logic;
  signal core_dmaaak1_n : std_logic;
  signal adapter_dma_io_cycle : std_logic;
  signal adapter_dma_io_active : std_logic;
  signal core_serial0_txb : byte_t := x"00";
  signal core_serial0_mode : byte_t := x"00";
  signal core_serial0_ctrl : byte_t := x"00";
  signal core_serial0_brg : byte_t := x"00";
  signal core_serial1_txb : byte_t := x"00";
  signal core_serial1_mode : byte_t := x"00";
  signal core_serial1_ctrl : byte_t := x"00";
  signal core_serial1_brg : byte_t := x"00";
  signal serial0_tx_tick_eff : std_logic;
  signal serial0_rx_tick_eff : std_logic;
  signal serial1_tx_tick_eff : std_logic;
  signal serial1_rx_tick_eff : std_logic;
  signal serial0_baud_tick : std_logic := '0';
  signal serial1_baud_tick : std_logic := '0';
  signal serial0_baud_prs_count : natural range 0 to 511 := 0;
  signal serial1_baud_prs_count : natural range 0 to 511 := 0;
  signal serial0_baud_brg_count : natural range 0 to 255 := 0;
  signal serial1_baud_brg_count : natural range 0 to 255 := 0;
  signal serial0_txd_r : std_logic := '1';
  signal serial0_tx_busy : std_logic := '0';
  signal serial0_tx_shift : byte_t := x"00";
  signal serial0_tx_bit_index : natural range 0 to 10 := 0;
  signal serial0_tx_data_bits : natural range 7 to 8 := 8;
  signal serial0_tx_parity_bits : natural range 0 to 1 := 0;
  signal serial0_tx_stop_bits : natural range 1 to 2 := 1;
  signal serial0_tx_parity_bit : std_logic := '0';
  signal serial0_tx_seen_txb : byte_t := x"00";
  signal serial0_tx_enable_prev : std_logic := '0';
  signal serial0_rx_sample_valid : std_logic := '0';
  signal serial0_rx_sample_data : byte_t := x"00";
  signal serial0_rx_sample_frame_error : std_logic := '0';
  signal serial0_rx_sample_parity_error : std_logic := '0';
  signal serial0_rx_busy : std_logic := '0';
  signal serial0_rx_shift : byte_t := x"00";
  signal serial0_rx_bit_index : natural range 0 to 10 := 0;
  signal serial0_rx_frame_bad : std_logic := '0';
  signal serial0_rx_parity_bad : std_logic := '0';
  signal serial0_rx_start_pending : std_logic := '0';
  signal serial0_rxd_prev : std_logic := '1';
  signal serial1_txd_r : std_logic := '1';
  signal serial1_tx_busy : std_logic := '0';
  signal serial1_tx_shift : byte_t := x"00";
  signal serial1_tx_bit_index : natural range 0 to 10 := 0;
  signal serial1_tx_data_bits : natural range 7 to 8 := 8;
  signal serial1_tx_parity_bits : natural range 0 to 1 := 0;
  signal serial1_tx_stop_bits : natural range 1 to 2 := 1;
  signal serial1_tx_parity_bit : std_logic := '0';
  signal serial1_tx_seen_txb : byte_t := x"00";
  signal serial1_tx_enable_prev : std_logic := '0';
  signal serial1_rx_sample_valid : std_logic := '0';
  signal serial1_rx_sample_data : byte_t := x"00";
  signal serial1_rx_sample_frame_error : std_logic := '0';
  signal serial1_rx_sample_parity_error : std_logic := '0';
  signal serial1_rx_busy : std_logic := '0';
  signal serial1_rx_shift : byte_t := x"00";
  signal serial1_rx_bit_index : natural range 0 to 10 := 0;
  signal serial1_rx_frame_bad : std_logic := '0';
  signal serial1_rx_parity_bad : std_logic := '0';
  signal serial1_rx_start_pending : std_logic := '0';
  signal serial1_rxd_prev : std_logic := '1';
  signal core_ram_control : byte_t := x"FC";
  signal core_halted : std_logic;
  signal core_stop_mode : std_logic;
  signal adapter_wait_states : unsigned(3 downto 0);
  signal ready_eff : std_logic;
  signal hold_req_eff : std_logic;
  signal adapter_hold_ack : std_logic;
  signal refresh_counter : unsigned(6 downto 0) := (others => '0');
  signal refresh_addr_counter : unsigned(8 downto 0) := (others => '0');
  signal refresh_pending : std_logic := '0';
  signal refresh_req : std_logic;
  signal refresh_ack : std_logic;
  signal refresh_active : std_logic;
  signal refresh_addr : addr20_t;
  signal refresh_wait_states : unsigned(3 downto 0);
  signal refresh_allowed : std_logic;
  signal refresh_timing_tick : std_logic := '0';

  function saturated_wait_sum(left : unsigned(3 downto 0); right : unsigned(3 downto 0))
    return unsigned is
    variable sum : unsigned(4 downto 0);
  begin
    sum := resize(left, 5) + resize(right, 5);
    if sum(4) = '1' then
      return "1111";
    end if;
    return sum(3 downto 0);
  end function;

  function rfm_refresh_wait(value : byte_t) return unsigned is
    variable result : unsigned(3 downto 0) := (others => '0');
  begin
    case value(3 downto 2) is
      when "00" =>
        result := "0000";
      when "01" =>
        result := "0001";
      when others =>
        result := "0010";
    end case;
    return result;
  end function;

  function rfm_interval_due(next_counter : unsigned(6 downto 0); value : byte_t)
    return boolean is
  begin
    case value(1 downto 0) is
      when "00" =>
        return next_counter(3 downto 0) = "0000";
      when "01" =>
        return next_counter(4 downto 0) = "00000";
      when "10" =>
        return next_counter(5 downto 0) = "000000";
      when others =>
        return next_counter = "0000000";
    end case;
  end function;

  function timer0_count_divisor(ctrl : byte_t) return natural is
  begin
    if ctrl(1 downto 0) = "01" then
      if ctrl(6) = '1' then
        return 128;
      end if;
      return 12;
    end if;

    if ctrl(6) = '1' then
      return 128;
    end if;
    return 6;
  end function;

  function timer0_md_divisor(ctrl : byte_t) return natural is
  begin
    if ctrl(4) = '1' then
      return 128;
    end if;
    return 12;
  end function;

  function timer1_divisor(ctrl : byte_t) return natural is
  begin
    if ctrl(6) = '1' then
      return 128;
    end if;
    return 6;
  end function;

  function serial_async_mode(mode : byte_t) return boolean is
  begin
    return mode(1 downto 0) = "01";
  end function;

  function serial_rx_data_bits(mode : byte_t) return natural is
  begin
    if serial_async_mode(mode) and mode(3) = '0' then
      return 7;
    end if;
    return 8;
  end function;

  function serial_rx_has_parity(mode : byte_t) return boolean is
  begin
    return serial_async_mode(mode) and mode(5 downto 4) /= "00";
  end function;

  function serial_rx_checks_parity(mode : byte_t) return boolean is
  begin
    return serial_async_mode(mode) and (mode(5 downto 4) = "10" or mode(5 downto 4) = "11");
  end function;

  function serial_rx_stop_bits(mode : byte_t) return natural is
  begin
    if serial_async_mode(mode) and mode(2) = '1' then
      return 2;
    end if;
    return 1;
  end function;

  function serial_prs_divisor(ctrl : byte_t) return natural is
    variable prs : natural := to_integer(unsigned(ctrl(3 downto 0)));
  begin
    if prs > 8 then
      prs := 8;
    end if;
    return 2 ** (prs + 1);
  end function;

  function serial_brg_divisor(brg : byte_t) return natural is
    variable value : natural := to_integer(unsigned(brg));
  begin
    if value < 2 then
      return 0;
    end if;
    return value;
  end function;

  function serial_odd_ones(value : byte_t; bits : natural) return std_logic is
    variable odd : std_logic := '0';
  begin
    for i in 0 to 7 loop
      if i < bits and value(i) = '1' then
        odd := not odd;
      end if;
    end loop;
    return odd;
  end function;

  function serial_tx_parity_bit(mode : byte_t; value : byte_t) return std_logic is
    variable odd : std_logic;
  begin
    odd := serial_odd_ones(value, serial_rx_data_bits(mode));
    case mode(5 downto 4) is
      when "01" =>
        return '0';
      when "10" =>
        return not odd;
      when "11" =>
        return odd;
      when others =>
        return '0';
    end case;
  end function;
begin
  adapter_wait_states <= saturated_wait_sum(wait_states, core_wait_states);
  adapter_dma_io_cycle <= not (core_dmaaak0_n and core_dmaaak1_n);
  dmaaak0_n_out <= core_dmaaak0_n when adapter_dma_io_active = '1' else '1';
  dmaaak1_n_out <= core_dmaaak1_n when adapter_dma_io_active = '1' else '1';
  serial0_txb <= core_serial0_txb;
  serial0_mode <= core_serial0_mode;
  serial0_ctrl <= core_serial0_ctrl;
  serial0_brg <= core_serial0_brg;
  serial1_txb <= core_serial1_txb;
  serial1_mode <= core_serial1_mode;
  serial1_ctrl <= core_serial1_ctrl;
  serial1_brg <= core_serial1_brg;
  timer0_control <= core_timer0_control;
  timer1_control <= core_timer1_control;
  serial0_txd_out <= serial0_txd_r;
  serial1_txd_out <= serial1_txd_r;
  serial0_sck_out <= serial0_tx_tick_eff when core_port1_mode_control(6) = '1' else '0';
  port0_mode_control <= core_port0_mode_control;
  port1_mode_control <= core_port1_mode_control;
  port2_mode_control <= core_port2_mode_control;
  clkout_out <= clk when core_port0_mode_control(7) = '1' else '0';
  timer0_tick_eff <= timer0_tick_in or timer0_prescale_tick when ENABLE_TIMER_PRESCALER else timer0_tick_in;
  timer0_md_tick_eff <= timer0_md_tick_in or timer0_md_prescale_tick when ENABLE_TIMER_PRESCALER else timer0_md_tick_in;
  timer1_tick_eff <= timer1_tick_in or timer1_prescale_tick when ENABLE_TIMER_PRESCALER else timer1_tick_in;
  serial0_tx_tick_eff <= serial0_tx_tick_in or serial0_baud_tick when ENABLE_SERIAL_BAUD else serial0_tx_tick_in;
  serial0_rx_tick_eff <= serial0_rx_tick_in or serial0_baud_tick when ENABLE_SERIAL_BAUD else serial0_rx_tick_in;
  serial1_tx_tick_eff <= serial1_tx_tick_in or serial1_baud_tick when ENABLE_SERIAL_BAUD else serial1_tx_tick_in;
  serial1_rx_tick_eff <= serial1_rx_tick_in or serial1_baud_tick when ENABLE_SERIAL_BAUD else serial1_rx_tick_in;
  ready_eff <= ready when core_port1_mode_control(7) = '1' else '1';
  hold_req_eff <= hold_req when core_port2_mode_control(7 downto 6) = "11" else '0';
  hold_ack <= adapter_hold_ack when core_port2_mode_control(7 downto 6) = "11" else '0';
  ram_control <= core_ram_control;
  halted <= core_halted;
  irq_request_eff <= irq_request when core_port1_mode_control(4) = '1' else '0';
  intak_n_out <= intak_n when core_port1_mode_control(3) = '1' else '1';
  int_ack_active_out <= (not intak_n) when core_port1_mode_control(3) = '1' else '0';
  int_ack_second_out <= core_int_ack_second when core_port1_mode_control(3) = '1' else '0';
  core_serial0_rx_valid <= serial0_rx_valid_in or serial0_rx_sample_valid;
  core_serial0_rx_data <= serial0_rx_data_in when serial0_rx_valid_in = '1' else serial0_rx_sample_data;
  core_serial0_rx_frame_error <= serial0_rx_frame_error_in when serial0_rx_valid_in = '1'
    else serial0_rx_sample_frame_error;
  core_serial0_rx_parity_error <= serial0_rx_parity_error_in when serial0_rx_valid_in = '1'
    else serial0_rx_sample_parity_error;
  core_serial1_rx_valid <= serial1_rx_valid_in or serial1_rx_sample_valid;
  core_serial1_rx_data <= serial1_rx_data_in when serial1_rx_valid_in = '1' else serial1_rx_sample_data;
  core_serial1_rx_frame_error <= serial1_rx_frame_error_in when serial1_rx_valid_in = '1'
    else serial1_rx_sample_frame_error;
  core_serial1_rx_parity_error <= serial1_rx_parity_error_in when serial1_rx_valid_in = '1'
    else serial1_rx_sample_parity_error;

  refresh_allowed <= '1' when core_ram_control(4) = '1' and
    (hold_req_eff = '0' or core_ram_control(6) = '1') and
    (core_halted = '0' or (core_stop_mode = '0' and core_ram_control(5) = '1')) else '0';
  refresh_req <= refresh_pending and refresh_allowed;
  refresh_addr <= std_logic_vector(resize(refresh_addr_counter, refresh_addr'length));
  refresh_wait_states <= rfm_refresh_wait(core_ram_control);
  refresh_active_out <= refresh_active;
  refresh_addr_out <= refresh_addr;
  refrq_n_out <= '0' when core_ram_control(7) = '0' else
    '1' when core_ram_control(4) = '0' else
    '0' when refresh_active = '1' else
    '1';

  core : entity work.v25_core
    generic map (
      RESET_PS => RESET_PS,
      RESET_IP => RESET_IP,
      RESET_BANK => RESET_BANK,
      ENABLE_TIMING_THROTTLE => ENABLE_TIMING_THROTTLE,
      FIXED_INSTRUCTION_BUDGET => FIXED_INSTRUCTION_BUDGET,
      ENABLE_PREFETCH_QUEUE => ENABLE_PREFETCH_QUEUE
    )
    port map (
      clk => clk,
      reset_n => reset_n,
      clock_enable => clock_enable,

      mem_valid => core_mem_valid,
      mem_write => core_mem_write,
      mem_addr => core_mem_addr,
      mem_wdata => core_mem_wdata,
      mem_rdata => core_mem_rdata,
      mem_ready => core_mem_ready,

      io_valid => core_io_valid,
      io_write => core_io_write,
      io_addr => core_io_addr,
      io_wdata => core_io_wdata,
      io_rdata => core_io_rdata,
      io_ready => core_io_ready,

      irq_request => irq_request_eff,
      irq_vector => irq_vector,
      int_ack_valid => core_int_ack_valid,
      int_ack_second => core_int_ack_second,
      int_ack_ready => core_int_ack_ready,
      int_ack_vector_data => core_int_ack_vector,
      int_ack_vector_valid => core_int_ack_vector_valid,
      nmi_in => nmi_in,
      intp0_in => intp0_in,
      intp1_in => intp1_in,
      intp2_in => intp2_in,

      serial0_rx_valid_in => core_serial0_rx_valid,
      serial0_rx_data_in => core_serial0_rx_data,
      serial0_rx_frame_error_in => core_serial0_rx_frame_error,
      serial0_rx_parity_error_in => core_serial0_rx_parity_error,
      serial0_rxd_in => serial0_rxd_in,
      serial1_rx_valid_in => core_serial1_rx_valid,
      serial1_rx_data_in => core_serial1_rx_data,
      serial1_rx_frame_error_in => core_serial1_rx_frame_error,
      serial1_rx_parity_error_in => core_serial1_rx_parity_error,
      serial1_rxd_in => serial1_rxd_in,

      port0_in => port0_in,
      port1_in => port1_in,
      port2_in => port2_in,
      portt_in => portt_in,
      port0_out => port0_out,
      port1_out => port1_out,
      port2_out => port2_out,
      portt_out => portt_out,
      port0_mode_control => core_port0_mode_control,
      port1_mode_control => core_port1_mode_control,
      port2_mode_control => core_port2_mode_control,
      timer0_tick_in => timer0_tick_eff,
      timer0_md_tick_in => timer0_md_tick_eff,
      timer1_tick_in => timer1_tick_eff,
      tout_out => core_tout,
      sfr_wait_states => core_wait_states,
      sfr_ready_extend => core_ready_extend,
      rfm_refresh_timing_in => refresh_timing_tick,
      dma0_control => dma0_control,
      dma0_mode => dma0_mode,
      dma0_start => dma0_start,
      dma0_hold => dma0_hold,
      dmarq0_in => dmarq0_in,
      dmaaak0_n_out => core_dmaaak0_n,
      tc0_n_out => tc0_n_out,
      dma1_control => dma1_control,
      dma1_mode => dma1_mode,
      dma1_start => dma1_start,
      dma1_hold => dma1_hold,
      dmarq1_in => dmarq1_in,
      dmaaak1_n_out => core_dmaaak1_n,
      tc1_n_out => tc1_n_out,
      serial0_rxb => serial0_rxb,
      serial0_txb => core_serial0_txb,
      serial0_mode => core_serial0_mode,
      serial0_ctrl => core_serial0_ctrl,
      serial0_brg => core_serial0_brg,
      serial0_enable => serial0_enable,
      serial1_rxb => serial1_rxb,
      serial1_txb => core_serial1_txb,
      serial1_mode => core_serial1_mode,
      serial1_ctrl => core_serial1_ctrl,
      serial1_brg => core_serial1_brg,
      serial1_enable => serial1_enable,
      standby_control => standby_control,
      ram_control => core_ram_control,
      flag_control => flag_control,
      protect_control => protect_control,
      timebase_control => timebase_control,
      interrupt_mode => interrupt_mode,
      interrupt_pending => interrupt_pending,
      ext_irq0_control => ext_irq0_control,
      ext_irq1_control => ext_irq1_control,
      ext_irq2_control => ext_irq2_control,
      timer0_irq_control => timer0_irq_control,
      timer1_irq_control => timer1_irq_control,
      timer2_irq_control => timer2_irq_control,
      timer0_control => core_timer0_control,
      timer1_control => core_timer1_control,
      dma0_irq_control => dma0_irq_control,
      dma1_irq_control => dma1_irq_control,
      serial0_error_irq_control => serial0_error_irq_control,
      serial0_rx_irq_control => serial0_rx_irq_control,
      serial0_tx_irq_control => serial0_tx_irq_control,
      serial1_error_irq_control => serial1_error_irq_control,
      serial1_rx_irq_control => serial1_rx_irq_control,
      serial1_tx_irq_control => serial1_tx_irq_control,

      halted => core_halted,
      stop_mode => core_stop_mode,
      fault => fault,

      debug_pc => debug_pc,
      debug_psw => debug_psw,
      debug_ax => debug_ax,
      debug_cx => debug_cx,
      debug_dx => debug_dx,
      debug_bx => debug_bx,
      debug_sp => debug_sp
    );

  bus_adapter_i : entity work.v25_bus_adapter
    generic map (
      MIN_WAIT_STATES => MIN_WAIT_STATES
    )
    port map (
      clk => clk,
      reset_n => reset_n,
      clock_enable => clock_enable,

      core_mem_valid => core_mem_valid,
      core_mem_write => core_mem_write,
      core_mem_addr => core_mem_addr,
      core_mem_wdata => core_mem_wdata,
      core_mem_rdata => core_mem_rdata,
      core_mem_ready => core_mem_ready,

      core_io_valid => core_io_valid,
      core_io_write => core_io_write,
      core_io_addr => core_io_addr,
      core_io_wdata => core_io_wdata,
      core_io_rdata => core_io_rdata,
      core_io_ready => core_io_ready,

      core_int_ack_valid => core_int_ack_valid,
      core_int_ack_second => core_int_ack_second,
      core_int_ack_ready => core_int_ack_ready,
      core_int_ack_vector => core_int_ack_vector,
      core_int_ack_vector_valid => core_int_ack_vector_valid,

      wait_states => adapter_wait_states,
      ready_extend => core_ready_extend,
      ready => ready_eff,
      dma_io_cycle => adapter_dma_io_cycle,
      dma_io_active => adapter_dma_io_active,
      hold_req => hold_req_eff,
      hold_ack => adapter_hold_ack,
      refresh_req => refresh_req,
      refresh_addr => refresh_addr,
      refresh_wait_states => refresh_wait_states,
      refresh_ack => refresh_ack,
      refresh_active => refresh_active,
      bus_addr => bus_addr,
      bus_dout => bus_dout,
      bus_din => bus_din,
      bus_doe => bus_doe,
      bus_r_w => bus_r_w,
      bus_mreq_n => bus_mreq_n,
      bus_mstb_n => bus_mstb_n,
      bus_iostb_n => bus_iostb_n,
      intak_n => intak_n
    );

  tout_out <= core_tout;

  refresh_i : process(clk)
    variable next_counter : unsigned(6 downto 0);
  begin
    if rising_edge(clk) then
      if reset_n = '0' then
        refresh_counter <= (others => '0');
        refresh_addr_counter <= (others => '0');
        refresh_pending <= '0';
        refresh_timing_tick <= '0';
      elsif clock_enable = '1' then
        next_counter := refresh_counter + 1;
        refresh_counter <= next_counter;
        refresh_timing_tick <= '0';

        if refresh_ack = '1' then
          refresh_pending <= '0';
          refresh_addr_counter <= refresh_addr_counter + 1;
        end if;

        if rfm_interval_due(next_counter, core_ram_control) then
          refresh_timing_tick <= '1';
          if refresh_allowed = '1' and refresh_pending = '0' then
            refresh_pending <= '1';
          end if;
        end if;
      end if;
    end if;
  end process;

  timer_prescale_i : process(clk)
    variable timer0_divisor_v : natural;
    variable timer0_md_divisor_v : natural;
    variable timer1_divisor_v : natural;
  begin
    if rising_edge(clk) then
      if reset_n = '0' then
        timer0_prescale_tick <= '0';
        timer0_md_prescale_tick <= '0';
        timer1_prescale_tick <= '0';
        timer0_prescale_count <= 0;
        timer0_md_prescale_count <= 0;
        timer1_prescale_count <= 0;
      elsif clock_enable = '1' then
        timer0_prescale_tick <= '0';
        timer0_md_prescale_tick <= '0';
        timer1_prescale_tick <= '0';

        if ENABLE_TIMER_PRESCALER then
          timer0_divisor_v := timer0_count_divisor(core_timer0_control);
          if timer0_prescale_count + 1 >= timer0_divisor_v then
            timer0_prescale_count <= 0;
            timer0_prescale_tick <= '1';
          else
            timer0_prescale_count <= timer0_prescale_count + 1;
          end if;

          timer0_md_divisor_v := timer0_md_divisor(core_timer0_control);
          if timer0_md_prescale_count + 1 >= timer0_md_divisor_v then
            timer0_md_prescale_count <= 0;
            timer0_md_prescale_tick <= '1';
          else
            timer0_md_prescale_count <= timer0_md_prescale_count + 1;
          end if;

          timer1_divisor_v := timer1_divisor(core_timer1_control);
          if timer1_prescale_count + 1 >= timer1_divisor_v then
            timer1_prescale_count <= 0;
            timer1_prescale_tick <= '1';
          else
            timer1_prescale_count <= timer1_prescale_count + 1;
          end if;
        else
          timer0_prescale_count <= 0;
          timer0_md_prescale_count <= 0;
          timer1_prescale_count <= 0;
        end if;
      end if;
    end if;
  end process;

  serial_baud_i : process(clk)
    variable prs_divisor_v : natural;
    variable brg_divisor_v : natural;
  begin
    if rising_edge(clk) then
      if reset_n = '0' then
        serial0_baud_tick <= '0';
        serial1_baud_tick <= '0';
        serial0_baud_prs_count <= 0;
        serial1_baud_prs_count <= 0;
        serial0_baud_brg_count <= 0;
        serial1_baud_brg_count <= 0;
      elsif clock_enable = '1' then
        serial0_baud_tick <= '0';
        serial1_baud_tick <= '0';

        if ENABLE_SERIAL_BAUD then
          prs_divisor_v := serial_prs_divisor(core_serial0_ctrl);
          brg_divisor_v := serial_brg_divisor(core_serial0_brg);
          if brg_divisor_v = 0 then
            serial0_baud_prs_count <= 0;
            serial0_baud_brg_count <= 0;
          elsif serial0_baud_prs_count + 1 >= prs_divisor_v then
            serial0_baud_prs_count <= 0;
            if serial0_baud_brg_count + 1 >= brg_divisor_v then
              serial0_baud_brg_count <= 0;
              serial0_baud_tick <= '1';
            else
              serial0_baud_brg_count <= serial0_baud_brg_count + 1;
            end if;
          else
            serial0_baud_prs_count <= serial0_baud_prs_count + 1;
          end if;

          prs_divisor_v := serial_prs_divisor(core_serial1_ctrl);
          brg_divisor_v := serial_brg_divisor(core_serial1_brg);
          if brg_divisor_v = 0 then
            serial1_baud_prs_count <= 0;
            serial1_baud_brg_count <= 0;
          elsif serial1_baud_prs_count + 1 >= prs_divisor_v then
            serial1_baud_prs_count <= 0;
            if serial1_baud_brg_count + 1 >= brg_divisor_v then
              serial1_baud_brg_count <= 0;
              serial1_baud_tick <= '1';
            else
              serial1_baud_brg_count <= serial1_baud_brg_count + 1;
            end if;
          else
            serial1_baud_prs_count <= serial1_baud_prs_count + 1;
          end if;
        else
          serial0_baud_prs_count <= 0;
          serial1_baud_prs_count <= 0;
          serial0_baud_brg_count <= 0;
          serial1_baud_brg_count <= 0;
        end if;
      end if;
    end if;
  end process;

  serial_tx_i : process(clk)
    variable stop_index_v : natural;
    variable serial0_cts_clear_v : boolean;
    variable serial1_cts_clear_v : boolean;
  begin
    if rising_edge(clk) then
      if reset_n = '0' then
        serial0_txd_r <= '1';
        serial0_tx_busy <= '0';
        serial0_tx_shift <= x"00";
        serial0_tx_bit_index <= 0;
        serial0_tx_data_bits <= 8;
        serial0_tx_parity_bits <= 0;
        serial0_tx_stop_bits <= 1;
        serial0_tx_parity_bit <= '0';
        serial0_tx_seen_txb <= x"00";
        serial0_tx_enable_prev <= '0';
        serial1_txd_r <= '1';
        serial1_tx_busy <= '0';
        serial1_tx_shift <= x"00";
        serial1_tx_bit_index <= 0;
        serial1_tx_data_bits <= 8;
        serial1_tx_parity_bits <= 0;
        serial1_tx_stop_bits <= 1;
        serial1_tx_parity_bit <= '0';
        serial1_tx_seen_txb <= x"00";
        serial1_tx_enable_prev <= '0';
      elsif clock_enable = '1' then
        serial0_cts_clear_v := (not serial_async_mode(core_serial0_mode)) or serial0_cts_in = '0';
        serial1_cts_clear_v := (not serial_async_mode(core_serial1_mode)) or serial1_cts_in = '0';

        if serial0_tx_busy = '0' then
          if core_serial0_mode(7) = '1' and serial0_cts_clear_v and
            (serial0_tx_enable_prev = '0' or core_serial0_txb /= serial0_tx_seen_txb) then
            serial0_tx_busy <= '1';
            serial0_tx_shift <= core_serial0_txb;
            serial0_tx_seen_txb <= core_serial0_txb;
            serial0_tx_bit_index <= 0;
            serial0_tx_data_bits <= serial_rx_data_bits(core_serial0_mode);
            if serial_rx_has_parity(core_serial0_mode) then
              serial0_tx_parity_bits <= 1;
            else
              serial0_tx_parity_bits <= 0;
            end if;
            serial0_tx_stop_bits <= serial_rx_stop_bits(core_serial0_mode);
            serial0_tx_parity_bit <= serial_tx_parity_bit(core_serial0_mode, core_serial0_txb);
            serial0_txd_r <= '0';
          else
            serial0_txd_r <= '1';
          end if;
        elsif serial0_tx_tick_eff = '1' and serial0_cts_clear_v then
          if serial0_tx_bit_index < serial0_tx_data_bits then
            serial0_txd_r <= serial0_tx_shift(serial0_tx_bit_index);
            serial0_tx_bit_index <= serial0_tx_bit_index + 1;
          elsif serial0_tx_parity_bits = 1 and serial0_tx_bit_index = serial0_tx_data_bits then
            serial0_txd_r <= serial0_tx_parity_bit;
            serial0_tx_bit_index <= serial0_tx_bit_index + 1;
          else
            stop_index_v := serial0_tx_bit_index - serial0_tx_data_bits - serial0_tx_parity_bits;
            serial0_txd_r <= '1';
            if stop_index_v + 1 >= serial0_tx_stop_bits then
              serial0_tx_busy <= '0';
            else
              serial0_tx_bit_index <= serial0_tx_bit_index + 1;
            end if;
          end if;
        end if;
        if serial0_cts_clear_v then
          serial0_tx_enable_prev <= core_serial0_mode(7);
        else
          serial0_tx_enable_prev <= '0';
        end if;

        if serial1_tx_busy = '0' then
          if core_serial1_mode(7) = '1' and serial1_cts_clear_v and
            (serial1_tx_enable_prev = '0' or core_serial1_txb /= serial1_tx_seen_txb) then
            serial1_tx_busy <= '1';
            serial1_tx_shift <= core_serial1_txb;
            serial1_tx_seen_txb <= core_serial1_txb;
            serial1_tx_bit_index <= 0;
            serial1_tx_data_bits <= serial_rx_data_bits(core_serial1_mode);
            if serial_rx_has_parity(core_serial1_mode) then
              serial1_tx_parity_bits <= 1;
            else
              serial1_tx_parity_bits <= 0;
            end if;
            serial1_tx_stop_bits <= serial_rx_stop_bits(core_serial1_mode);
            serial1_tx_parity_bit <= serial_tx_parity_bit(core_serial1_mode, core_serial1_txb);
            serial1_txd_r <= '0';
          else
            serial1_txd_r <= '1';
          end if;
        elsif serial1_tx_tick_eff = '1' and serial1_cts_clear_v then
          if serial1_tx_bit_index < serial1_tx_data_bits then
            serial1_txd_r <= serial1_tx_shift(serial1_tx_bit_index);
            serial1_tx_bit_index <= serial1_tx_bit_index + 1;
          elsif serial1_tx_parity_bits = 1 and serial1_tx_bit_index = serial1_tx_data_bits then
            serial1_txd_r <= serial1_tx_parity_bit;
            serial1_tx_bit_index <= serial1_tx_bit_index + 1;
          else
            stop_index_v := serial1_tx_bit_index - serial1_tx_data_bits - serial1_tx_parity_bits;
            serial1_txd_r <= '1';
            if stop_index_v + 1 >= serial1_tx_stop_bits then
              serial1_tx_busy <= '0';
            else
              serial1_tx_bit_index <= serial1_tx_bit_index + 1;
            end if;
          end if;
        end if;
        if serial1_cts_clear_v then
          serial1_tx_enable_prev <= core_serial1_mode(7);
        else
          serial1_tx_enable_prev <= '0';
        end if;
      end if;
    end if;
  end process;

  serial_rx_i : process(clk)
    variable data_bits_v : natural;
    variable parity_bits_v : natural;
    variable stop_bits_v : natural;
    variable stop_index_v : natural;
    variable frame_error_v : std_logic;
    variable total_odd_v : std_logic;
  begin
    if rising_edge(clk) then
      if reset_n = '0' then
        serial0_rx_sample_valid <= '0';
        serial0_rx_sample_data <= x"00";
        serial0_rx_sample_frame_error <= '0';
        serial0_rx_sample_parity_error <= '0';
        serial0_rx_busy <= '0';
        serial0_rx_shift <= x"00";
        serial0_rx_bit_index <= 0;
        serial0_rx_frame_bad <= '0';
        serial0_rx_parity_bad <= '0';
        serial0_rx_start_pending <= '0';
        serial0_rxd_prev <= '1';
        serial1_rx_sample_valid <= '0';
        serial1_rx_sample_data <= x"00";
        serial1_rx_sample_frame_error <= '0';
        serial1_rx_sample_parity_error <= '0';
        serial1_rx_busy <= '0';
        serial1_rx_shift <= x"00";
        serial1_rx_bit_index <= 0;
        serial1_rx_frame_bad <= '0';
        serial1_rx_parity_bad <= '0';
        serial1_rx_start_pending <= '0';
        serial1_rxd_prev <= '1';
      elsif clock_enable = '1' then
        serial0_rx_sample_valid <= '0';
        serial0_rx_sample_frame_error <= '0';
        serial0_rx_sample_parity_error <= '0';
        serial1_rx_sample_valid <= '0';
        serial1_rx_sample_frame_error <= '0';
        serial1_rx_sample_parity_error <= '0';

        if ENABLE_SERIAL_BAUD then
          if core_serial0_mode(6) = '1' and serial0_rx_busy = '0' and serial0_rx_start_pending = '0' and
            serial0_rxd_prev = '1' and serial0_rxd_in = '0' then
            serial0_rx_start_pending <= '1';
          end if;
          if core_serial1_mode(6) = '1' and serial1_rx_busy = '0' and serial1_rx_start_pending = '0' and
            serial1_rxd_prev = '1' and serial1_rxd_in = '0' then
            serial1_rx_start_pending <= '1';
          end if;
        else
          serial0_rx_start_pending <= '0';
          serial1_rx_start_pending <= '0';
        end if;

        if serial0_rx_tick_eff = '1' then
          data_bits_v := serial_rx_data_bits(core_serial0_mode);
          if serial_rx_has_parity(core_serial0_mode) then
            parity_bits_v := 1;
          else
            parity_bits_v := 0;
          end if;
          stop_bits_v := serial_rx_stop_bits(core_serial0_mode);

          if serial0_rx_busy = '0' then
            if ENABLE_SERIAL_BAUD and serial0_baud_tick = '1' and serial0_rx_tick_in = '0' then
              if serial0_rx_start_pending = '1' and serial0_rxd_in = '0' then
                serial0_rx_busy <= '1';
                serial0_rx_shift <= x"00";
                serial0_rx_bit_index <= 0;
                serial0_rx_frame_bad <= '0';
                serial0_rx_parity_bad <= '0';
              end if;
              serial0_rx_start_pending <= '0';
            elsif serial0_rxd_in = '0' then
              serial0_rx_busy <= '1';
              serial0_rx_shift <= x"00";
              serial0_rx_bit_index <= 0;
              serial0_rx_frame_bad <= '0';
              serial0_rx_parity_bad <= '0';
            end if;
          elsif serial0_rx_bit_index < data_bits_v then
            serial0_rx_shift(serial0_rx_bit_index) <= serial0_rxd_in;
            serial0_rx_bit_index <= serial0_rx_bit_index + 1;
          elsif parity_bits_v = 1 and serial0_rx_bit_index = data_bits_v then
            total_odd_v := serial_odd_ones(serial0_rx_shift, data_bits_v) xor serial0_rxd_in;
            if serial_rx_checks_parity(core_serial0_mode) then
              if (core_serial0_mode(5 downto 4) = "10" and total_odd_v = '0') or
                (core_serial0_mode(5 downto 4) = "11" and total_odd_v = '1') then
                serial0_rx_parity_bad <= '1';
              else
                serial0_rx_parity_bad <= '0';
              end if;
            else
              serial0_rx_parity_bad <= '0';
            end if;
            serial0_rx_bit_index <= serial0_rx_bit_index + 1;
          else
            stop_index_v := serial0_rx_bit_index - data_bits_v - parity_bits_v;
            frame_error_v := serial0_rx_frame_bad or (not serial0_rxd_in);
            if stop_index_v + 1 >= stop_bits_v then
              serial0_rx_busy <= '0';
              serial0_rx_sample_data <= serial0_rx_shift;
              serial0_rx_sample_valid <= '1';
              serial0_rx_sample_frame_error <= frame_error_v;
              serial0_rx_sample_parity_error <= serial0_rx_parity_bad;
            else
              serial0_rx_frame_bad <= frame_error_v;
              serial0_rx_bit_index <= serial0_rx_bit_index + 1;
            end if;
          end if;
        end if;

        if serial1_rx_tick_eff = '1' then
          data_bits_v := serial_rx_data_bits(core_serial1_mode);
          if serial_rx_has_parity(core_serial1_mode) then
            parity_bits_v := 1;
          else
            parity_bits_v := 0;
          end if;
          stop_bits_v := serial_rx_stop_bits(core_serial1_mode);

          if serial1_rx_busy = '0' then
            if ENABLE_SERIAL_BAUD and serial1_baud_tick = '1' and serial1_rx_tick_in = '0' then
              if serial1_rx_start_pending = '1' and serial1_rxd_in = '0' then
                serial1_rx_busy <= '1';
                serial1_rx_shift <= x"00";
                serial1_rx_bit_index <= 0;
                serial1_rx_frame_bad <= '0';
                serial1_rx_parity_bad <= '0';
              end if;
              serial1_rx_start_pending <= '0';
            elsif serial1_rxd_in = '0' then
              serial1_rx_busy <= '1';
              serial1_rx_shift <= x"00";
              serial1_rx_bit_index <= 0;
              serial1_rx_frame_bad <= '0';
              serial1_rx_parity_bad <= '0';
            end if;
          elsif serial1_rx_bit_index < data_bits_v then
            serial1_rx_shift(serial1_rx_bit_index) <= serial1_rxd_in;
            serial1_rx_bit_index <= serial1_rx_bit_index + 1;
          elsif parity_bits_v = 1 and serial1_rx_bit_index = data_bits_v then
            total_odd_v := serial_odd_ones(serial1_rx_shift, data_bits_v) xor serial1_rxd_in;
            if serial_rx_checks_parity(core_serial1_mode) then
              if (core_serial1_mode(5 downto 4) = "10" and total_odd_v = '0') or
                (core_serial1_mode(5 downto 4) = "11" and total_odd_v = '1') then
                serial1_rx_parity_bad <= '1';
              else
                serial1_rx_parity_bad <= '0';
              end if;
            else
              serial1_rx_parity_bad <= '0';
            end if;
            serial1_rx_bit_index <= serial1_rx_bit_index + 1;
          else
            stop_index_v := serial1_rx_bit_index - data_bits_v - parity_bits_v;
            frame_error_v := serial1_rx_frame_bad or (not serial1_rxd_in);
            if stop_index_v + 1 >= stop_bits_v then
              serial1_rx_busy <= '0';
              serial1_rx_sample_data <= serial1_rx_shift;
              serial1_rx_sample_valid <= '1';
              serial1_rx_sample_frame_error <= frame_error_v;
              serial1_rx_sample_parity_error <= serial1_rx_parity_bad;
            else
              serial1_rx_frame_bad <= frame_error_v;
              serial1_rx_bit_index <= serial1_rx_bit_index + 1;
            end if;
          end if;
        end if;

        serial0_rxd_prev <= serial0_rxd_in;
        serial1_rxd_prev <= serial1_rxd_in;
      end if;
    end if;
  end process;
end architecture;
