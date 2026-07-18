-- SPDX-License-Identifier: BSD-3-Clause
--
-- External bus timing adapter for the experimental NEC V25-compatible core.
--
-- The CPU core uses an abstract byte ready/valid memory and I/O interface. This
-- adapter turns one abstract transfer into a simple V25-style external bus
-- cycle with a T1 address/strobe phase, a programmable minimum wait interval,
-- READY extension, and a T2 completion phase. It is intentionally a pin-timing
-- building block, not a prefetch or instruction-cycle model.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.v25_pkg.all;

entity v25_bus_adapter is
  generic (
    MIN_WAIT_STATES : natural := 0
  );
  port (
    clk     : in std_logic;
    reset_n : in std_logic;
    clock_enable : in std_logic := '1';

    core_mem_valid : in  std_logic;
    core_mem_write : in  std_logic;
    core_mem_addr  : in  addr20_t;
    core_mem_wdata : in  byte_t;
    core_mem_rdata : out byte_t;
    core_mem_ready : out std_logic;

    core_io_valid : in  std_logic;
    core_io_write : in  std_logic;
    core_io_addr  : in  word_t;
    core_io_wdata : in  byte_t;
    core_io_rdata : out byte_t;
    core_io_ready : out std_logic;

    core_int_ack_valid : in std_logic := '0';
    core_int_ack_second : in std_logic := '0';
    core_int_ack_ready : out std_logic;
    core_int_ack_vector : out byte_t := x"00";
    core_int_ack_vector_valid : out std_logic := '0';

    wait_states : in unsigned(3 downto 0) := (others => '0');
    ready_extend : in std_logic := '1';
    ready    : in std_logic;
    dma_io_cycle : in std_logic := '0';
    dma_io_active : out std_logic := '0';
    hold_req : in std_logic := '0';
    hold_ack : out std_logic;

    refresh_req         : in  std_logic := '0';
    refresh_addr        : in  addr20_t := (others => '0');
    refresh_wait_states : in  unsigned(3 downto 0) := (others => '0');
    refresh_ack         : out std_logic;
    refresh_active      : out std_logic;

    bus_addr  : out addr20_t;
    bus_dout  : out byte_t;
    bus_din   : in  byte_t;
    bus_doe   : out std_logic;
    bus_r_w   : out std_logic;
    bus_mreq_n : out std_logic;
    bus_mstb_n : out std_logic;
    bus_iostb_n : out std_logic;
    intak_n : out std_logic
  );
end entity;

architecture rtl of v25_bus_adapter is
  type bus_state_t is (
    BUS_IDLE,
    BUS_T1,
    BUS_WAIT,
    BUS_T2,
    BUS_RECOVER,
    BUS_HOLD,
    BUS_REFRESH_T1,
    BUS_REFRESH_WAIT,
    BUS_REFRESH_T2,
    BUS_REFRESH_RECOVER,
    BUS_INTACK_T1,
    BUS_INTACK_WAIT,
    BUS_INTACK_T2,
    BUS_INTACK_RECOVER
  );

  signal state : bus_state_t := BUS_IDLE;
  signal wait_count : natural := 0;
  signal is_io_r : std_logic := '0';
  signal write_r : std_logic := '0';
  signal addr_r : addr20_t := (others => '0');
  signal wdata_r : byte_t := x"00";
  signal mem_rdata_r : byte_t := x"00";
  signal io_rdata_r : byte_t := x"00";
  signal mem_ready_r : std_logic := '0';
  signal io_ready_r : std_logic := '0';
  signal int_ack_ready_r : std_logic := '0';
  signal int_ack_vector_r : byte_t := x"00";
  signal int_ack_vector_valid_r : std_logic := '0';
  signal bus_active : std_logic;
  signal refresh_active_r : std_logic;
  signal int_ack_active_r : std_logic;
  signal refresh_ack_r : std_logic := '0';
  signal wait_states_r : natural range 0 to 15 := 0;
  signal ready_extend_r : std_logic := '1';
  signal dma_io_cycle_r : std_logic := '0';
  signal int_ack_second_r : std_logic := '0';
  signal int_ack_sequence_r : std_logic := '0';
  signal refresh_wait_states_r : natural range 0 to 15 := 0;
  signal refresh_addr_r : addr20_t := (others => '0');
begin
  bus_active <= '1' when state = BUS_T1 or state = BUS_WAIT or state = BUS_T2 else '0';
  refresh_active_r <= '1' when state = BUS_REFRESH_T1 or state = BUS_REFRESH_WAIT or state = BUS_REFRESH_T2 else '0';
  int_ack_active_r <= '1' when state = BUS_INTACK_T1 or state = BUS_INTACK_WAIT or state = BUS_INTACK_T2 else '0';
  bus_addr <= refresh_addr_r when refresh_active_r = '1' else addr_r;
  bus_dout <= wdata_r;
  bus_doe <= '1' when bus_active = '1' and write_r = '1' else '0';
  bus_r_w <= '1' when refresh_active_r = '1' or int_ack_active_r = '1' else not write_r;
  bus_mreq_n <= '0' when bus_active = '1' and is_io_r = '0' else '1';
  bus_mstb_n <= '0' when bus_active = '1' and is_io_r = '0' else '1';
  bus_iostb_n <= '0' when bus_active = '1' and is_io_r = '1' and dma_io_cycle_r = '0' else '1';
  dma_io_active <= '1' when bus_active = '1' and is_io_r = '1' and dma_io_cycle_r = '1' else '0';
  intak_n <= '0' when int_ack_active_r = '1' else '1';
  hold_ack <= '1' when state = BUS_HOLD else '0';
  refresh_ack <= refresh_ack_r;
  refresh_active <= refresh_active_r;

  core_mem_rdata <= mem_rdata_r;
  core_io_rdata <= io_rdata_r;
  core_mem_ready <= mem_ready_r;
  core_io_ready <= io_ready_r;
  core_int_ack_ready <= int_ack_ready_r;
  core_int_ack_vector <= int_ack_vector_r;
  core_int_ack_vector_valid <= int_ack_vector_valid_r;

  process(clk)
  begin
    if rising_edge(clk) then
      if reset_n = '0' then
        mem_ready_r <= '0';
        io_ready_r <= '0';
        int_ack_ready_r <= '0';
        int_ack_vector_valid_r <= '0';
        refresh_ack_r <= '0';
        state <= BUS_IDLE;
        wait_count <= 0;
        is_io_r <= '0';
        write_r <= '0';
        addr_r <= (others => '0');
        wdata_r <= x"00";
        mem_rdata_r <= x"00";
        io_rdata_r <= x"00";
        int_ack_ready_r <= '0';
        int_ack_vector_r <= x"00";
        int_ack_vector_valid_r <= '0';
        wait_states_r <= 0;
        ready_extend_r <= '1';
        dma_io_cycle_r <= '0';
        int_ack_second_r <= '0';
        int_ack_sequence_r <= '0';
        refresh_wait_states_r <= 0;
        refresh_addr_r <= (others => '0');
      elsif clock_enable = '1' then
        mem_ready_r <= '0';
        io_ready_r <= '0';
        int_ack_ready_r <= '0';
        int_ack_vector_valid_r <= '0';
        refresh_ack_r <= '0';
        case state is
          when BUS_IDLE =>
            wait_count <= 0;
            if int_ack_sequence_r = '1' then
              if core_int_ack_valid = '1' then
                int_ack_second_r <= core_int_ack_second;
                state <= BUS_INTACK_T1;
              end if;
            elsif refresh_req = '1' then
              refresh_addr_r <= refresh_addr;
              refresh_wait_states_r <= to_integer(refresh_wait_states);
              state <= BUS_REFRESH_T1;
            elsif hold_req = '1' then
              state <= BUS_HOLD;
            elsif core_int_ack_valid = '1' then
              int_ack_second_r <= core_int_ack_second;
              state <= BUS_INTACK_T1;
            elsif core_mem_valid = '1' then
              is_io_r <= '0';
              write_r <= core_mem_write;
              addr_r <= core_mem_addr;
              wdata_r <= core_mem_wdata;
              wait_states_r <= to_integer(wait_states);
              ready_extend_r <= ready_extend;
              dma_io_cycle_r <= '0';
              state <= BUS_T1;
            elsif core_io_valid = '1' then
              is_io_r <= '1';
              write_r <= core_io_write;
              addr_r <= x"0" & core_io_addr;
              wdata_r <= core_io_wdata;
              wait_states_r <= to_integer(wait_states);
              ready_extend_r <= ready_extend;
              dma_io_cycle_r <= dma_io_cycle;
              state <= BUS_T1;
            end if;

          when BUS_T1 =>
            wait_count <= 0;
            state <= BUS_WAIT;

          when BUS_WAIT =>
            if wait_count < MIN_WAIT_STATES + wait_states_r then
              wait_count <= wait_count + 1;
            elsif ready_extend_r = '0' or ready = '1' then
              state <= BUS_T2;
            end if;

          when BUS_T2 =>
            if is_io_r = '1' then
              if write_r = '0' then
                io_rdata_r <= bus_din;
              end if;
              io_ready_r <= '1';
            else
              if write_r = '0' then
                mem_rdata_r <= bus_din;
              end if;
              mem_ready_r <= '1';
            end if;
            state <= BUS_RECOVER;

          when BUS_RECOVER =>
            if core_mem_valid = '0' and core_io_valid = '0' and core_int_ack_valid = '0' then
              if hold_req = '1' then
                state <= BUS_HOLD;
              else
                state <= BUS_IDLE;
              end if;
            end if;

          when BUS_HOLD =>
            if refresh_req = '1' then
              refresh_addr_r <= refresh_addr;
              refresh_wait_states_r <= to_integer(refresh_wait_states);
              wait_count <= 0;
              state <= BUS_REFRESH_T1;
            elsif hold_req = '0' then
              state <= BUS_IDLE;
            end if;

          when BUS_REFRESH_T1 =>
            wait_count <= 0;
            state <= BUS_REFRESH_WAIT;

          when BUS_REFRESH_WAIT =>
            if wait_count < refresh_wait_states_r then
              wait_count <= wait_count + 1;
            else
              state <= BUS_REFRESH_T2;
            end if;

          when BUS_REFRESH_T2 =>
            refresh_ack_r <= '1';
            state <= BUS_REFRESH_RECOVER;

          when BUS_REFRESH_RECOVER =>
            if refresh_req = '0' then
              if hold_req = '1' then
                state <= BUS_HOLD;
              elsif core_int_ack_valid = '1' then
                int_ack_second_r <= core_int_ack_second;
                wait_count <= 0;
                state <= BUS_INTACK_T1;
              else
                state <= BUS_IDLE;
              end if;
            end if;

          when BUS_INTACK_T1 =>
            wait_count <= 0;
            state <= BUS_INTACK_WAIT;

          when BUS_INTACK_WAIT =>
            if int_ack_second_r = '1' and wait_count < 5 then
              wait_count <= wait_count + 1;
            else
              state <= BUS_INTACK_T2;
            end if;

          when BUS_INTACK_T2 =>
            if int_ack_second_r = '1' then
              int_ack_vector_r <= bus_din;
              int_ack_vector_valid_r <= '1';
              int_ack_sequence_r <= '0';
            else
              int_ack_sequence_r <= '1';
            end if;
            int_ack_ready_r <= '1';
            state <= BUS_INTACK_RECOVER;

          when BUS_INTACK_RECOVER =>
            if core_int_ack_valid = '0' then
              if int_ack_sequence_r = '1' then
                state <= BUS_IDLE;
              elsif hold_req = '1' then
                state <= BUS_HOLD;
              else
                state <= BUS_IDLE;
              end if;
            end if;
        end case;
      end if;
    end if;
  end process;
end architecture;
